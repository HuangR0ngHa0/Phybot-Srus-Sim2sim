from __future__ import annotations

import copy
import queue
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional

import numpy as np


class NavMode(str, Enum):
    STANDBY = "STANDBY"
    LOW_SPEED = "LOW_SPEED"
    MEDIUM_SPEED = "MEDIUM_SPEED"
    EMERGENCY = "EMERGENCY"


@dataclass(frozen=True)
class ModeDecision:
    mode: NavMode
    run_policy: bool
    render_depth: bool
    force_zero: bool
    vx_max: float
    wz_max: float
    zero_reason_override: Optional[str] = None
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


_DECISIONS = {
    NavMode.STANDBY: ModeDecision(
        mode=NavMode.STANDBY,
        run_policy=False,
        render_depth=False,
        force_zero=True,
        vx_max=0.0,
        wz_max=0.0,
        zero_reason_override="standby",
        reset_adapter=True,
        zero_burst_count=3,
        preserve_policy_goal_dist=False,
    ),
    NavMode.LOW_SPEED: ModeDecision(
        mode=NavMode.LOW_SPEED,
        run_policy=True,
        render_depth=True,
        force_zero=False,
        vx_max=0.6,
        wz_max=0.8,
        zero_reason_override=None,
        reset_adapter=False,
        zero_burst_count=0,
        preserve_policy_goal_dist=False,
    ),
    NavMode.MEDIUM_SPEED: ModeDecision(
        mode=NavMode.MEDIUM_SPEED,
        run_policy=True,
        render_depth=True,
        force_zero=False,
        vx_max=1.0,
        wz_max=1.3,
        zero_reason_override=None,
        reset_adapter=False,
        zero_burst_count=0,
        preserve_policy_goal_dist=False,
    ),
    NavMode.EMERGENCY: ModeDecision(
        mode=NavMode.EMERGENCY,
        run_policy=True,
        render_depth=True,
        force_zero=True,
        vx_max=0.0,
        wz_max=0.0,
        zero_reason_override="emergency",
        reset_adapter=False,
        zero_burst_count=0,
        preserve_policy_goal_dist=True,
    ),
}

_KEY_TO_MODE = {
    "A": NavMode.STANDBY,
    "S": NavMode.LOW_SPEED,
    "D": NavMode.MEDIUM_SPEED,
    "F": NavMode.EMERGENCY,
    "G": NavMode.STANDBY,
}


class NavModeController:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._mode = NavMode.STANDBY
        self._status: dict[str, Any] = {}
        now = time.time()
        self._last_event = LastEvent(
            key="",
            accepted=True,
            previous_mode=NavMode.STANDBY,
            new_mode=NavMode.STANDBY,
            message="init",
            timestamp=now,
        )
        self._events: queue.Queue[LastEvent] = queue.Queue()
        self._input_stop = threading.Event()
        self._input_thread: Optional[threading.Thread] = None
        self._input_prompt = "NavSide> "

    def get_mode(self) -> NavMode:
        with self._lock:
            return self._mode

    def get_decision(self) -> ModeDecision:
        return _DECISIONS[self.get_mode()]

    def get_last_event(self) -> LastEvent:
        with self._lock:
            return self._last_event

    def update_status(self, **kwargs: Any) -> None:
        with self._lock:
            self._status.update(kwargs)

    def set_status(self, snapshot: dict[str, Any]) -> None:
        with self._lock:
            self._status = copy.deepcopy(snapshot) if snapshot is not None else {}

    def apply_line(self, line: str) -> LastEvent:
        text = "" if line is None else str(line).strip()
        key = text[:1].upper() if text else ""
        now = time.time()

        with self._lock:
            previous_mode = self._mode
            new_mode = previous_mode
            accepted = False
            message = "ignored"

            if key in _KEY_TO_MODE:
                new_mode = _KEY_TO_MODE[key]
                accepted = True
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
                self._mode = new_mode
            elif text == "":
                message = "empty_input"
            else:
                message = "invalid_key"

            event = LastEvent(
                key=key,
                accepted=accepted,
                previous_mode=previous_mode,
                new_mode=new_mode,
                message=message,
                timestamp=now,
            )
            self._last_event = event
            self._events.put(event)
            return event

    def poll_events(self) -> list[LastEvent]:
        events: list[LastEvent] = []
        while True:
            try:
                events.append(self._events.get_nowait())
            except queue.Empty:
                break
        return events

    def start_input_thread(self, prompt: str = "NavSide> ") -> threading.Thread:
        with self._lock:
            if self._input_thread is not None and self._input_thread.is_alive():
                return self._input_thread
            self._input_prompt = prompt
            self._input_stop.clear()
            thread = threading.Thread(
                target=self._input_loop,
                name="NavModeInputThread",
                daemon=True,
            )
            self._input_thread = thread

        thread.start()
        return thread

    def stop_input_thread(self) -> None:
        self._input_stop.set()
        thread = None
        with self._lock:
            thread = self._input_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=0.2)

    def render_panel(self) -> str:
        with self._lock:
            mode = self._mode
            status = copy.deepcopy(self._status)
            last_event = self._last_event

        decision = _DECISIONS[mode]
        final_cmd = self._format_vec(status.get("final_cmd"))
        policy_cmd = self._format_vec(status.get("policy_cmd", status.get("raw_action")))
        zero_reason = self._format_scalar(status.get("zero_reason"))
        goal_dist = self._format_scalar(
            status.get("policy_goal_dist", status.get("goal_dist"))
        )
        state_source = self._format_scalar(status.get("state_source"))
        robot_xy = self._format_robot_xy(status.get("robot_xy"))
        last_event_text = self._format_last_event(last_event)

        lines = [
            "=== NavSide Mode Panel ===",
            f"mode: {mode.value}",
            f"network: {'running' if decision.run_policy else 'stopped'}",
            f"output: {'forced_zero' if decision.force_zero else 'normal'}",
            f"limits: vx_max={decision.vx_max:.3f} wz_max={decision.wz_max:.3f}",
            f"final_cmd: {final_cmd}",
            f"policy_cmd: {policy_cmd}",
            f"zero_reason: {zero_reason}",
            f"policy_goal_dist / goal_dist: {goal_dist}",
            f"state_source: {state_source}",
            f"robot_xy: {robot_xy}",
            f"last_event: {last_event_text}",
            "keys: A->STANDBY | S->LOW_SPEED | D->MEDIUM_SPEED | F->EMERGENCY | G->STANDBY(quit_to_standby)",
        ]
        return "\033[H\033[J" + "\n".join(lines) + "\n"

    def _input_loop(self) -> None:
        while not self._input_stop.is_set():
            try:
                line = input(self._input_prompt)
            except (EOFError, KeyboardInterrupt, OSError):
                break
            self.apply_line(line)

    @staticmethod
    def _format_scalar(value: Any) -> str:
        if value is None:
            return "n/a"
        if isinstance(value, str):
            return value
        if isinstance(value, (bool, np.bool_)):
            return "true" if bool(value) else "false"
        if isinstance(value, (int, float, np.integer, np.floating)):
            return f"{float(value):.3f}"
        arr = np.asarray(value)
        if arr.size == 0:
            return "n/a"
        if arr.size == 1:
            return f"{float(arr.reshape(-1)[0]):.3f}"
        return np.array2string(arr, precision=3, separator=", ")

    @staticmethod
    def _format_vec(value: Any) -> str:
        if value is None:
            return "n/a"
        arr = np.asarray(value, dtype=np.float32).reshape(-1)
        if arr.size == 0:
            return "n/a"
        return np.array2string(arr, precision=3, separator=", ")

    @staticmethod
    def _format_robot_xy(value: Any) -> str:
        if value is None:
            return "n/a"
        arr = np.asarray(value, dtype=np.float32).reshape(-1)
        if arr.size < 2:
            return "n/a"
        return f"({arr[0]:.3f}, {arr[1]:.3f})"

    @staticmethod
    def _format_last_event(event: LastEvent) -> str:
        return (
            f"key={event.key or 'n/a'} accepted={str(event.accepted).lower()} "
            f"{event.previous_mode.value}->{event.new_mode.value} "
            f"message={event.message} ts={event.timestamp:.3f}"
        )


__all__ = [
    "LastEvent",
    "ModeDecision",
    "NavMode",
    "NavModeController",
]
