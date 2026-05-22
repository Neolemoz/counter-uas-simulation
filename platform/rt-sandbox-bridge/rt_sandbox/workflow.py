"""Session-scoped multi-step sandbox workflows (PLAT-RT-S6)."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from typing import Any

from rt_sandbox.template_catalog import get_template
from rt_sandbox.templates import TemplateApplyResult, apply_template
from rt_sandbox.world_state import WorldStateStore

WORKFLOW_SCHEMA = "rt_sandbox_workflow_v1"


class WorkflowStatus(StrEnum):
    IDLE = "idle"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass(frozen=True)
class WorkflowStep:
    kind: str  # reset_world | apply_template | ready
    template_id: str | None = None


@dataclass(frozen=True)
class WorkflowDefinition:
    workflow_id: str
    description: str
    steps: tuple[WorkflowStep, ...]


@dataclass
class WorkflowState:
    workflow_id: str
    status: WorkflowStatus
    current_step: int
    step_count: int
    templates_applied: list[str] = field(default_factory=list)
    last_error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": WORKFLOW_SCHEMA,
            "workflow_id": self.workflow_id,
            "status": self.status.value,
            "current_step": self.current_step,
            "step_count": self.step_count,
            "templates_applied": list(self.templates_applied),
            "last_error": self.last_error,
        }


_BUILTIN_WORKFLOWS: tuple[WorkflowDefinition, ...] = (
    WorkflowDefinition(
        workflow_id="staging_radar_then_drones_v1",
        description="Reset world, apply radar preset, apply drone pair, ready",
        steps=(
            WorkflowStep(kind="reset_world"),
            WorkflowStep(kind="apply_template", template_id="radar_valley_pair_v1"),
            WorkflowStep(kind="apply_template", template_id="drone_patrol_pair_v1"),
            WorkflowStep(kind="ready"),
        ),
    ),
    WorkflowDefinition(
        workflow_id="waypoint_staging_v1",
        description="Reset, waypoint triangle, ready",
        steps=(
            WorkflowStep(kind="reset_world"),
            WorkflowStep(kind="apply_template", template_id="waypoint_patrol_triangle_v1"),
            WorkflowStep(kind="ready"),
        ),
    ),
    WorkflowDefinition(
        workflow_id="minimal_radar_demo_v1",
        description="North arc radar staging without reset",
        steps=(
            WorkflowStep(kind="apply_template", template_id="radar_north_arc_v1"),
            WorkflowStep(kind="ready"),
        ),
    ),
)

_WORKFLOW_BY_ID: dict[str, WorkflowDefinition] = {w.workflow_id: w for w in _BUILTIN_WORKFLOWS}


def list_workflow_ids() -> list[str]:
    return sorted(_WORKFLOW_BY_ID.keys())


def get_workflow(workflow_id: str) -> WorkflowDefinition | None:
    return _WORKFLOW_BY_ID.get(workflow_id)


def list_workflows_metadata() -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for w in sorted(_BUILTIN_WORKFLOWS, key=lambda x: x.workflow_id):
        out.append(
            {
                "workflow_id": w.workflow_id,
                "description": w.description,
                "step_count": len(w.steps),
                "steps": [
                    {
                        "kind": s.kind,
                        "template_id": s.template_id,
                    }
                    for s in w.steps
                ],
            }
        )
    return out


def catalog_size() -> int:
    return len(_WORKFLOW_BY_ID)


def validate_workflow_payload(payload: Any) -> str | None:
    if not isinstance(payload, dict):
        return "INVALID_STATE"
    workflow_id = payload.get("workflow_id")
    if not isinstance(workflow_id, str) or not workflow_id:
        return "INVALID_STATE"
    if get_workflow(workflow_id) is None:
        return "INVALID_STATE"
    return None


def new_workflow_state(workflow_id: str) -> WorkflowState | None:
    wf = get_workflow(workflow_id)
    if wf is None:
        return None
    return WorkflowState(
        workflow_id=workflow_id,
        status=WorkflowStatus.IN_PROGRESS,
        current_step=0,
        step_count=len(wf.steps),
    )


@dataclass
class StepExecutionResult:
    ok: bool
    transition: str
    step_index: int
    staged_setup: bool = False
    template_apply: TemplateApplyResult | None = None
    world_reset: bool = False
    error_code: str | None = None
    completed: bool = False


def execute_workflow_step(
    state: WorkflowState,
    world: WorldStateStore,
    *,
    max_entities_per_apply: int,
) -> StepExecutionResult:
    """Execute current step; mutates state and world."""
    wf = get_workflow(state.workflow_id)
    if wf is None:
        state.status = WorkflowStatus.FAILED
        state.last_error = "unknown workflow"
        return StepExecutionResult(
            ok=False,
            transition="workflow_failed",
            step_index=state.current_step,
            error_code="INVALID_STATE",
        )
    if state.current_step >= len(wf.steps):
        state.status = WorkflowStatus.COMPLETED
        return StepExecutionResult(
            ok=True,
            transition="workflow_already_completed",
            step_index=state.current_step,
            completed=True,
        )

    step = wf.steps[state.current_step]
    idx = state.current_step

    if step.kind == "reset_world":
        world.reset()
        state.current_step += 1
        if state.current_step >= len(wf.steps):
            state.status = WorkflowStatus.COMPLETED
            return StepExecutionResult(
                ok=True,
                transition="reset_world",
                step_index=idx,
                world_reset=True,
                completed=state.status == WorkflowStatus.COMPLETED,
            )
        return StepExecutionResult(
            ok=True,
            transition="reset_world",
            step_index=idx,
            world_reset=True,
        )

    if step.kind == "apply_template":
        tid = step.template_id
        if not tid or get_template(tid) is None:
            state.status = WorkflowStatus.FAILED
            state.last_error = "invalid template in workflow"
            return StepExecutionResult(
                ok=False,
                transition="workflow_step_failed",
                step_index=idx,
                error_code="WORKFLOW_STEP_FAILED",
            )
        result, err = apply_template(
            world,
            tid,
            max_entities_per_apply=max_entities_per_apply,
        )
        if err:
            state.status = WorkflowStatus.FAILED
            state.last_error = err
            return StepExecutionResult(
                ok=False,
                transition="workflow_step_failed",
                step_index=idx,
                staged_setup=True,
                error_code="WORKFLOW_STEP_FAILED",
            )
        assert result is not None
        state.templates_applied.append(tid)
        state.current_step += 1
        if state.current_step >= len(wf.steps):
            state.status = WorkflowStatus.COMPLETED
        return StepExecutionResult(
            ok=True,
            transition="apply_template",
            step_index=idx,
            staged_setup=True,
            template_apply=result,
            completed=state.status == WorkflowStatus.COMPLETED,
        )

    if step.kind == "ready":
        state.status = WorkflowStatus.COMPLETED
        state.current_step += 1
        return StepExecutionResult(
            ok=True,
            transition="ready",
            step_index=idx,
            completed=True,
        )

    state.status = WorkflowStatus.FAILED
    state.last_error = f"unknown step kind: {step.kind}"
    return StepExecutionResult(
        ok=False,
        transition="workflow_step_failed",
        step_index=idx,
        error_code="WORKFLOW_STEP_FAILED",
    )
