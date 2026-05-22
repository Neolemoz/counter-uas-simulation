"""Runtime backend abstraction: stub or Gazebo adapter (PLAT-RT-G2)."""

from __future__ import annotations

from typing import Any, Protocol

from rt_sandbox.governance import GovernanceConfig
from rt_sandbox.runtime_stub import RuntimeStub


class RuntimeHandle(Protocol):
    kind: str

    def start(self) -> int: ...

    def is_alive(self) -> bool: ...

    def pause(self) -> None: ...

    def resume(self) -> None: ...

    def stop(self) -> None: ...

    def terminate(self) -> None: ...

    def kill_for_crash_simulation(self) -> None: ...

    def health_payload(self) -> dict[str, Any]: ...

    def apply_pose(
        self,
        entity_id: str,
        entity_type: str,
        pose: dict[str, float],
    ) -> dict[str, Any] | None: ...

    def delete_entity(self, entity_id: str) -> dict[str, Any] | None: ...

    def reset_world(self) -> None: ...


def create_runtime(
    config: GovernanceConfig,
    session_id: str,
) -> RuntimeHandle:
    if config.enable_gazebo_adapter:
        from rt_sandbox.runtime_adapter import GazeboRuntimeAdapter

        return GazeboRuntimeAdapter(
            session_id=session_id,
            mode=config.adapter_mode,
            ipc_timeout_s=config.adapter_ipc_timeout_s,
            ready_timeout_s=config.adapter_ready_timeout_s,
            ros_domain_id=config.ros_domain_id_for_session(session_id),
        )
    return RuntimeStub()


def runtime_is_adapter(runtime: RuntimeHandle) -> bool:
    return getattr(runtime, "kind", "") == "adapter"
