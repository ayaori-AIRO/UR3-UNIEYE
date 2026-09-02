"""Reusable synchronous XML-RPC client for the installed WEISS GRIPKIT daemon."""

import time
import xmlrpc.client

from ur3_moveit_examples.weiss_gripper_read_state import (
    DEFAULT_DEVICE_ID,
    DEFAULT_URL,
    STATE_NAMES,
    TimeoutTransport,
)


COMMUNICATION_FAULT = -1
NOT_REFERENCED = 0
IDLE = 1
RELEASED = 2
NO_PART = 4
HOLDING = 8


class WeissGripperError(RuntimeError):
    """Raised when communication or a gripper operation fails."""


class WeissGripper:
    def __init__(
        self,
        *,
        url: str = DEFAULT_URL,
        device_id: str = DEFAULT_DEVICE_ID,
        connection_timeout: float = 3.0,
        operation_timeout: float = 10.0,
        poll_interval: float = 0.1,
    ) -> None:
        self.url = url
        self.device_id = device_id
        self.operation_timeout = operation_timeout
        self.poll_interval = poll_interval
        self._proxy = xmlrpc.client.ServerProxy(
            url,
            transport=TimeoutTransport(connection_timeout),
            allow_none=True,
            use_builtin_types=True,
        )

    def close(self) -> None:
        self._proxy.__exit__(None, None, None)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    @staticmethod
    def state_name(state: int) -> str:
        return STATE_NAMES.get(state, "UNKNOWN")

    def get_state(self) -> int:
        state = self._proxy.GetState(self.device_id)
        if not isinstance(state, int):
            raise WeissGripperError(f"GetState returned invalid value {state!r}")
        if state == COMMUNICATION_FAULT:
            raise WeissGripperError("gripper reports COMMUNICATION_FAULT")
        return state

    def get_position(self) -> float:
        position = self._proxy.GetPos(self.device_id)
        if not isinstance(position, (int, float)):
            raise WeissGripperError(f"GetPos returned invalid value {position!r}")
        return float(position)

    def require_released(self) -> None:
        state = self.get_state()
        if state != RELEASED:
            raise WeissGripperError(
                "scenario requires RELEASED(2) before moving to Point 1; "
                f"current state={state} ({self.state_name(state)})"
            )

    def grip(self, index: int = 0) -> int:
        self.require_released()
        self._proxy.Grip(self.device_id, index)
        return self._poll_for((NO_PART, HOLDING), "Grip")

    def release(self, index: int = 0) -> int:
        state = self.get_state()
        if state == NOT_REFERENCED:
            raise WeissGripperError("Release blocked: gripper is NOT_REFERENCED")
        if state == RELEASED:
            return state
        self._proxy.Release(self.device_id, index)
        return self._poll_for((RELEASED,), "Release")

    def _poll_for(self, terminal_states: tuple[int, ...], operation: str) -> int:
        deadline = time.monotonic() + self.operation_timeout
        while time.monotonic() < deadline:
            state = self.get_state()
            if state == NOT_REFERENCED:
                raise WeissGripperError(
                    f"{operation} failed: gripper became NOT_REFERENCED"
                )
            if state in terminal_states:
                return state
            time.sleep(self.poll_interval)
        state = self.get_state()
        raise WeissGripperError(
            f"{operation} timed out; state={state} ({self.state_name(state)}), "
            f"position={self.get_position():.3f} mm"
        )
