# SPDX-License-Identifier: MulanPSL-2.0
"""Backend over the official AUBO Python SDK (``pyaubo_sdk`` 0.26).

Connecting never changes power, brakes, speed fraction, runtime state, TCP,
payload or safety configuration.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import threading

from .backend import CLIENT_INTERFACES, RpcError, check_allowed, require_zero


def load_credentials(path):
    """Read ``{"username", "password"}`` from a private (0600) JSON file."""
    path = Path(path)
    if path.stat().st_mode & 0o077:
        raise RpcError("SDK credentials file must not be readable by group or others")
    auth = json.loads(path.read_text())
    if not isinstance(auth, dict) or not {"username", "password"} <= set(auth):
        raise RpcError("SDK credentials file must hold username and password")
    return auth


class SdkClient:
    """One authenticated RPC connection; writes are refused unless ``motion``."""

    def __init__(self, host, port=30004, robot="rob1", timeout_s=1.0, motion=False,
                 credentials_path=None):
        """Connect and log in with a bounded request timeout.

        Credentials come from ``credentials_path`` or ``AUBO_RPC_CREDENTIALS``
        and are cleared from memory after login. Any failure closes the
        connection before raising.
        """
        import pyaubo_sdk

        if not 0 < timeout_s <= 5:
            raise ValueError("SDK request timeout must be in (0, 5] seconds")
        path = credentials_path or os.environ.get("AUBO_RPC_CREDENTIALS")
        if not path:
            raise RpcError("AUBO_RPC_CREDENTIALS is not set")
        self.motion = motion is True
        self._lock = threading.Lock()
        self._rpc = pyaubo_sdk.RpcClient()
        self._rpc.setRequestTimeout(int(timeout_s * 1000))
        self._robot = None
        auth = {}
        try:
            auth = load_credentials(path)
            self._rpc.connect(host, port)
            if not self._rpc.hasConnected():
                raise RpcError(f"SDK connection to {host}:{port} failed")
            self._rpc.login(auth["username"], auth["password"])
            if not self._rpc.hasLogined():
                raise RpcError("SDK authentication failed")
            if robot not in self._rpc.getRobotNames():
                raise RpcError(f"controller has no robot named {robot}")
            self._robot = self._rpc.getRobotInterface(robot)
        except Exception:
            self.close()
            raise
        finally:
            auth.clear()

    def call(self, method, params=()):
        """Invoke one allowlisted ARCS method and normalize the reply.

        SDK enums become their names and tuples become lists so every backend
        returns the same wire shapes.
        """
        check_allowed(method, self.motion)
        with self._lock:
            if self._robot is None:
                raise RpcError("SDK connection is closed")
            if method == "getRobotNames":
                return list(self._rpc.getRobotNames())
            group, name = method.split(".")
            parent = self._rpc if group in CLIENT_INTERFACES else self._robot
            try:
                interface = getattr(parent, f"get{group}")()
                result = getattr(interface, name)(*params)
            except Exception as exc:
                raise RpcError(f"{method}: SDK request failed ({type(exc).__name__}: {exc})") from exc
        if hasattr(result, "name") and not isinstance(result, (str, int, float)):
            return result.name
        return list(result) if isinstance(result, tuple) else result

    def command(self, method, params):
        """Send one write; never retried, because it may already have executed."""
        require_zero(method, self.call(method, params))

    def close(self):
        """Log out and disconnect this connection only; never power the arm off."""
        rpc, self._robot = self._rpc, None
        try:
            if rpc.hasLogined():
                rpc.logout()
        finally:
            if rpc.hasConnected():
                rpc.disconnect()
