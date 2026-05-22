"""Loopback-only HTTP bridge server."""

from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

from rt_sandbox.session_manager import BridgeSessionManager

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 18765


class _BridgeHandler(BaseHTTPRequestHandler):
    manager: BridgeSessionManager

    def log_message(self, format: str, *args: Any) -> None:
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path in ("/v1/telemetry/pull", "/v1/telemetry/pull/"):
            self._handle_telemetry_pull(parsed)
            return
        self._json_response(404, {"error_code": "NOT_FOUND", "ok": False})

    def do_POST(self) -> None:
        if self.path not in ("/v1/command", "/v1/command/"):
            self._json_response(404, {"error_code": "NOT_FOUND", "ok": False})
            return
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            body = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            self._json_response(400, {"error_code": "INVALID_REQUEST", "ok": False})
            return
        result = self.manager.handle_command(body)
        self._json_response(200, result)

    def _handle_telemetry_pull(self, parsed: Any) -> None:
        qs = parse_qs(parsed.query)
        session_id = (qs.get("session_id") or [None])[0]
        subscription_id = (qs.get("subscription_id") or [None])[0]
        max_events_raw = (qs.get("max_events") or ["10"])[0]
        if not session_id or not subscription_id:
            self._json_response(
                400,
                {
                    "ok": False,
                    "error_code": "INVALID_REQUEST",
                    "message": "session_id and subscription_id required",
                },
            )
            return
        try:
            max_events = max(1, min(64, int(max_events_raw)))
        except (TypeError, ValueError):
            max_events = 10
        result = self.manager.pull_telemetry(session_id, subscription_id, max_events)
        status = 200 if result.get("ok") else 404
        self._json_response(status, result)

    def _json_response(self, status: int, payload: dict[str, Any]) -> None:
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def make_server(
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    manager: BridgeSessionManager | None = None,
    repo_root: Path | None = None,
) -> ThreadingHTTPServer:
    if host not in ("127.0.0.1", "localhost", "::1"):
        raise ValueError("RT bridge must bind to loopback only")
    mgr = manager or BridgeSessionManager(repo_root=repo_root)
    handler = type("BoundBridgeHandler", (_BridgeHandler,), {"manager": mgr})
    return ThreadingHTTPServer((host, port), handler)


def serve_forever(
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    repo_root: Path | None = None,
) -> None:
    server = make_server(host=host, port=port, repo_root=repo_root)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()


def serve_in_background(
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    repo_root: Path | None = None,
) -> tuple[ThreadingHTTPServer, Any]:
    import threading

    server = make_server(host=host, port=port, repo_root=repo_root)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread
