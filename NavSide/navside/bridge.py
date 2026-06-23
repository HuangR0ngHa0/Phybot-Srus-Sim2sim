import socket
import struct
import threading
from dataclasses import dataclass

import numpy as np

from .state import SruRobotState


SRU2_MAGIC = 0x32555253


@dataclass
class LegacyStatePacket:
    x: float
    y: float
    yaw: float
    source: str = "legacy_3f"


@dataclass
class NavStatePacketV2:
    seq: int
    timestamp_sec: float
    linear_vel_b: np.ndarray
    angular_vel_b: np.ndarray
    projected_gravity_b: np.ndarray
    robot_pos_w: np.ndarray
    robot_quat_wxyz: np.ndarray
    source: str = "nav_state_v2"

    def to_sru_robot_state(self) -> SruRobotState:
        return SruRobotState(
            linear_vel_b=self.linear_vel_b.astype(np.float32),
            angular_vel_b=self.angular_vel_b.astype(np.float32),
            projected_gravity_b=self.projected_gravity_b.astype(np.float32),
            robot_pos_w=self.robot_pos_w.astype(np.float32),
            robot_quat_wxyz=self.robot_quat_wxyz.astype(np.float32),
        )


class RobotComm(threading.Thread):
    """UDP bridge compatible with the existing robot-side CPG runtime."""

    def __init__(
        self,
        local_ip: str = "127.0.0.1",
        local_port: int = 8081,
        remote_ip: str = "127.0.0.1",
        remote_port: int = 8080,
    ):
        super().__init__()
        self.cmd_packer = struct.Struct("3f")
        self.legacy_state_unpacker = struct.Struct("3f")
        self.nav_state_v2_unpacker = struct.Struct("<IHHId16f")
        self.local_addr = (local_ip, local_port)
        self.remote_addr = (remote_ip, remote_port)
        self.latest_state = None
        self.lock = threading.Lock()
        self.running = True

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.local_addr)
        self.sock.settimeout(0.05)
        print(f"[UDP] NavSide listening on {local_ip}:{local_port}")
        print(f"[UDP] NavSide sending to {remote_ip}:{remote_port}")

    def run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
                state = self._parse_state_packet(data)
                if state is not None:
                    with self.lock:
                        self.latest_state = state
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as exc:
                print(f"[UDP] receive error: {exc}")
                break

    def get_latest_state(self):
        with self.lock:
            return self.latest_state

    def _parse_state_packet(self, data: bytes):
        if len(data) == self.nav_state_v2_unpacker.size:
            unpacked = self.nav_state_v2_unpacker.unpack(data)
            magic, version, _flags, seq, timestamp_sec = unpacked[:5]
            if magic != SRU2_MAGIC or version != 2:
                print(f"[UDP] invalid NavStatePacketV2 header: magic={magic:#x} version={version}")
                return None

            values = np.asarray(unpacked[5:], dtype=np.float32)
            return NavStatePacketV2(
                seq=int(seq),
                timestamp_sec=float(timestamp_sec),
                linear_vel_b=values[0:3].copy(),
                angular_vel_b=values[3:6].copy(),
                projected_gravity_b=values[6:9].copy(),
                robot_pos_w=values[9:12].copy(),
                robot_quat_wxyz=values[12:16].copy(),
            )

        if len(data) == self.legacy_state_unpacker.size:
            x, y, yaw = self.legacy_state_unpacker.unpack(data)
            return LegacyStatePacket(float(x), float(y), float(yaw))

        print(f"[UDP] unexpected state packet size: {len(data)} bytes")
        return None

    def send_command(self, vx: float, vy: float, wz: float):
        packet = self.cmd_packer.pack(float(vx), float(vy), float(wz))
        self.sock.sendto(packet, self.remote_addr)

    def send_zero(self):
        self.send_command(0.0, 0.0, 0.0)

    def stop(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass
