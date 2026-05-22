"""JSON-lines IPC between bridge and adapter worker subprocess."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any


@dataclass
class IpcRequest:
    op: str
    request_id: str
    session_id: str
    payload: dict[str, Any] = field(default_factory=dict)

    def to_line(self) -> str:
        body = {
            "schema": "rt_adapter_ipc_request_v1",
            "request_id": self.request_id,
            "op": self.op,
            "session_id": self.session_id,
            "payload": self.payload,
        }
        return json.dumps(body, separators=(",", ":")) + "\n"


@dataclass
class IpcResponse:
    request_id: str
    ok: bool
    result: dict[str, Any] = field(default_factory=dict)
    error_code: str | None = None
    error_message: str | None = None

    @classmethod
    def from_line(cls, line: str) -> IpcResponse:
        data = json.loads(line)
        return cls(
            request_id=str(data.get("request_id", "")),
            ok=bool(data.get("ok")),
            result=dict(data.get("result") or {}),
            error_code=data.get("error_code"),
            error_message=data.get("error_message"),
        )

    def to_line(self) -> str:
        body: dict[str, Any] = {
            "schema": "rt_adapter_ipc_response_v1",
            "request_id": self.request_id,
            "ok": self.ok,
            "result": self.result,
        }
        if self.error_code:
            body["error_code"] = self.error_code
        if self.error_message:
            body["error_message"] = self.error_message
        return json.dumps(body, separators=(",", ":")) + "\n"
