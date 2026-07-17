from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Dict, List, Optional

import numpy as np


class NavMode(Enum):
    STANDBY = auto()
    LOW_SPEED = auto()
    MEDIUM_SPEED = auto()
    EMERGENCY = auto()


@dataclass(frozen=True)
class ModeDecision:
    mode: NavMode
    run_policy: bool
    render_depth: bool
    force_zero: bool
    vx_max: float
    wz_max: float
    zero_reason_override: str = ""
    reset_adapter: bool = False
    zero_burst_count: int = 0
    preserve_policy_goal_dist: bool = False


@dataclass(frozen=True)
class LastEvent:
    key: str
    accepted: bool
    previous_mode: NavMode
    new_mode: NavMode
    message: str
    timestamp: float


@dataclass
class _StatusSnapshot:
    final_cmd: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    policy_cmd: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    goal_dist: Optional[float] = None
    zero_reason: str = ""
    state_source: str = ""
    robot_xy: Optional[np.ndarray] = None
    extra: Dict[str, Any] = field(default_factory=dict)


class NavModeController:
    """Thread-safe navigation mode controller for future NavSide integration.

    Design choice:
    - `apply_line()` mutates the controller and returns the last event.
    - `poll_events()` drains the event queue so a future UI loop can refresh
      without coupling to input handling.
    """

    _KEY_TO_MODE = {
        "A": NavMode.STANDBY,
        "S": NavMode.LOW_SPEED,
        "D": NavMode.MEDIUM_SPEED,
        "F": NavMode.EMERGENCY,
        "G": NavMode.STANDBY,
    }

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._mode = NavMode.STANDBY
        self._last_event: Optional[LastEvent] = None
        self._status = _StatusSnapshot()
        self._event_queue: "queue.SimpleQueue[LastEvent]" = queue.SimpleQueue()
        self._stop_event = threading.Event()
        self._input_thread: Optional[threading.Thread] = None
        self._input_prompt = "NavSide> "

    def _decision_for_mode(self, mode: NavMode) -> ModeDecision:
        if mode == NavMode.STANDBY:
            return ModeDecision(
                mode=mode,
                run_policy=False,
                render_depth=False,
                force_zero=True,
                vx_max=0.0,
                wz_max=0.0,
                zero_reason_override="standby",
                reset_adapter=True,
                zero_burst_count=3,
                preserve_policy_goal_dist=False,
            )
        if mode == NavMode.LOW_SPEED:
            return ModeDecision(
                mode=mode,
                run_policy=True,
                render_depth=True,
                force_zero=False,
                vx_max=0.6,
                wz_max=0.8,
            )
        if mode == NavMode.MEDIUM_SPEED:
            return ModeDecision(
                mode=mode,
                run_policy=True,
                render_depth=True,
                force_zero=False,
                vx_max=1.0,
                wz_max=1.3,
            )
        if mode == NavMode.EMERGENCY:
            return ModeDecision(
                mode=mode,
                run_policy=True,
                render_depth=True,
                force_zero=True,
                vx_max=0.0,
                wz_max=0.0,
                zero_reason_override="emergency",
                reset_adapter=False,
                zero_burst_count=0,
                preserve_policy_goal_dist=True,
            )
        raise ValueError(f"Unsupported NavMode: {mode}")

    def _make_event(
        self,
        key: str,
        accepted: bool,
        previous_mode: NavMode,
        new_mode: NavMode,
        message: str,
    ) -> LastEvent:
        event = LastEvent(
            key=key,
            accepted=accepted,
            previous_mode=previous_mode,
            new_mode=new_mode,
            message=message,
            timestamp=time.time(),
        )
        with self._lock:
            self._last_event = event
        self._event_queue.put(event)
        return event

    @staticmethod
    def _normalize_key(line: str) -> str:
        return line.strip().upper()

    def apply_line(self, line: str) -> LastEvent:
        normalized = self._normalize_key("" if line is None else line)
        previous_mode = self.get_mode()

        if not normalized:
            return self._make_event(
                key="",
                accepted=False,
                previous_mode=previous_mode,
                new_mode=previous_mode,
                message="empty_input",
            )

        key = normalized[0]
        if normalized != key or key not in self._KEY_TO_MODE:
            return self._make_event(
                key=normalized,
                accepted=False,
                previous_mode=previous_mode,
                new_mode=previous_mode,
                message=f"invalid_key:{normalized}",
            )

        new_mode = self._KEY_TO_MODE[key]
        with self._lock:
            self._mode = new_mode

        if key == "G":
            message = "quit_to_standby"
        elif key == "A":
            message = "standby"
        elif key == "S":
            message = "low_speed"
        elif key == "D":
            message = "medium_speed"
        elif key == "F":
            message = "emergency"
        else:
            message = new_mode.name.lower()

        return self._make_event(
            key=key,
            accepted=True,
            previous_mode=previous_mode,
            new_mode=new_mode,
            message=message,
        )

    def get_mode(self) -> NavMode:
        with self._lock:
            return self._mode

    def get_decision(self) -> ModeDecision:
        return self._decision_for_mode(self.get_mode())

    def get_last_event(self) -> Optional[LastEvent]:
        with self._lock:
            return self._last_event

    def update_status(self, **kwargs: Any) -> None:
        with self._lock:
            for key, value in kwargs.items():
                if hasattr(self._status, key):
                    setattr(self._status, key, value)
                else:
                    self._status.extra[key] = value

    def set_status(self, snapshot: Dict[str, Any]) -> None:
        with self._lock:
            self._status = _StatusSnapshot()
            for key, value in snapshot.items():
                if hasattr(self._status, key):
                    setattr(self._status, key, value)
                else:
                    self._status.extra[key] = value

    def _snapshot_status(self) -> _StatusSnapshot:
        with self._lock:
            return _StatusSnapshot(
                final_cmd=np.asarray(self._status.final_cmd, dtype=np.float32).copy(),
                policy_cmd=np.asarray(self._status.policy_cmd, dtype=np.float32).copy(),
                goal_dist=self._status.goal_dist,
                zero_reason=str(self._status.zero_reason),
                state_source=str(self._status.state_source),
                robot_xy=None
                if self._status.robot_xy is None
                else np.asarray(self._status.robot_xy, dtype=np.float32).copy(),
                extra=dict(self._status.extra),
            )

    @staticmethod
    def _fmt_vec(value: Any, precision: int = 4) -> str:
        if value is None:
            return "N/A"
        arr = np.asarray(value, dtype=np.float32).reshape(-1)
        return np.array2string(arr, precision=precision)

    def render_panel(self) -> str:
        decision = self.get_decision()
        status = self._snapshot_status()
        last_event = self.get_last_event()
        mode = decision.mode.name
        network_state = "running" if decision.run_policy else "stopped"
        output_state = "forced_zero" if decision.force_zero else "normal"
        last_event_text = "N/A"
        if last_event is not None:
            last_event_text = (
                f"{last_event.key} accepted={last_event.accepted} "
                f"{last_event.previous_mode.name}->{last_event.new_mode.name} "
                f"{last_event.message}"
            )

        lines = [
            "\033[H\033[J",
            "[NavSide Mode Panel]",
            f"mode: {mode}",
            f"network: {network_state}",
            f"output: {output_state}",
            f"limits: vx_max={decision.vx_max:.3f} wz_max={decision.wz_max:.3f}",
            f"final_cmd: {self._fmt_vec(status.final_cmd)}",
            f"policy_cmd: {self._fmt_vec(status.policy_cmd)}",
            f"zero_reason: {status.zero_reason or 'N/A'}",
            f"policy_goal_dist: {status.goal_dist if status.goal_dist is not None else 'N/A'}",
            f"state_source: {status.state_source or 'N/A'}",
            f"robot_xy: {self._fmt_vec(status.robot_xy)}",
            f"last_event: {last_event_text}",
            "keys: A=standby S=low D=medium F=emergency G=standby(q_to_standby)",
        ]
        if status.extra:
            lines.append(f"extra: {status.extra}")
        return "\n".join(lines)

    def start_input_thread(self, prompt: str = "NavSide> ") -> None:
        with self._lock:
            thread = self._input_thread
            if thread is not None and thread.is_alive():
                return
            self._stop_event.clear()
            self._input_prompt = prompt

            def _run() -> None:
                while not self._stop_event.is_set():
                    try:
                        line = input(self._input_prompt)
                    except (EOFError, KeyboardInterrupt, OSError):
                        break
                    self.apply_line(line)

            self._input_thread = threading.Thread(
                target=_run,
                name="NavModeInput",
                daemon=True,
            )
            self._input_thread.start()

    def stop_input_thread(self) -> None:
        self._stop_event.set()
        thread = self._input_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.2)

    def poll_events(self) -> List[LastEvent]:
        events: List[LastEvent] = []
        while True:
            try:
                events.append(self._event_queue.get_nowait())
            except queue.Empty:
                break
        return events

