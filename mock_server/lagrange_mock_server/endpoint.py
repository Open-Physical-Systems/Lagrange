"""Endpoint strings, the same forms as lagrange::proto::parseEndpoint."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

DEFAULT_TCP_PORT = 47610


@dataclass(frozen=True)
class Endpoint:
    kind: str  # "unix" or "tcp"
    address: str
    port: int = 0

    def __str__(self) -> str:
        if self.kind == "unix":
            return f"unix:{self.address}"
        host = f"[{self.address}]" if ":" in self.address else self.address
        return f"tcp://{host}:{self.port}"


def parse_endpoint(text: str) -> Endpoint:
    """Parse "unix:/path", "/path", "tcp://host[:port]" or "tcp://[v6]:port"."""
    if text.startswith("unix:") and len(text) > 5:
        return Endpoint("unix", text[5:])
    if text.startswith("/"):
        return Endpoint("unix", text)
    if not text.startswith("tcp://"):
        raise ValueError(f"not an endpoint: {text!r}")
    rest = text[6:]
    port_text = ""
    if rest.startswith("["):
        close = rest.find("]")
        if close < 0:
            raise ValueError(f"not an endpoint: {text!r}")
        host, tail = rest[1:close], rest[close + 1 :]
        if tail:
            if not tail.startswith(":") or len(tail) == 1:
                raise ValueError(f"not an endpoint: {text!r}")
            port_text = tail[1:]
    elif ":" in rest:
        host, port_text = rest.rsplit(":", 1)
        if not port_text:
            raise ValueError(f"not an endpoint: {text!r}")
    else:
        host = rest
    if not host:
        raise ValueError(f"not an endpoint: {text!r}")
    port = DEFAULT_TCP_PORT
    if port_text:
        if not port_text.isdigit() or not 0 < int(port_text) <= 65535:
            raise ValueError(f"bad port in endpoint: {text!r}")
        port = int(port_text)
    return Endpoint("tcp", host, port)


def default_unix_socket_path(instance: str = "default") -> str:
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    directory = Path(runtime) / "lagrange" if runtime else Path(f"/tmp/lagrange-{os.getuid()}")
    return str(directory / f"{instance}.sock")
