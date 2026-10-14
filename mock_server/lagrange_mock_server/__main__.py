"""Command line of the mock server.

lagrange_mock_server --sample                       serve the built-in synthetic sample
lagrange_mock_server --recording run.mcap           serve a recording
lagrange_mock_server --urdf robot.urdf              show a robot description (joints at zero)
lagrange_mock_server --write-sample sample.mcap     write the sample to a file and exit
lagrange_mock_server --check unix:/path/to.sock     run the conformance check against a server
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import sys
from pathlib import Path

from . import __version__
from .client import run_conformance
from .endpoint import default_unix_socket_path, parse_endpoint
from .recording import RecordingError, read_recording, write_recording
from .replay import ReplayEngine
from .sample import generate_sample
from .server import MockServer
from .urdf import UrdfError, import_urdf


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="lagrange_mock_server",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--version", action="version", version=__version__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--recording", type=Path, help="MCAP recording to serve")
    source.add_argument("--sample", action="store_true", help="serve the synthetic sample")
    source.add_argument("--urdf", type=Path, help="URDF or xacro file to show (joints at zero)")
    source.add_argument(
        "--write-sample",
        type=Path,
        metavar="PATH",
        help="write the synthetic sample to PATH and exit",
    )
    source.add_argument(
        "--check",
        metavar="ENDPOINT",
        help="run the conformance check against a running server and exit",
    )
    parser.add_argument(
        "--listen",
        default=f"unix:{default_unix_socket_path()}",
        help="endpoint to listen on (default: %(default)s)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="length of the synthetic sample in seconds (default: %(default)s)",
    )
    parser.add_argument("--seed", type=int, default=1, help="seed of the synthetic sample")
    parser.add_argument(
        "--no-loop", action="store_true", help="pause at the end instead of looping"
    )
    parser.add_argument("--paused", action="store_true", help="start paused")
    parser.add_argument("--verbose", action="store_true")
    return parser


async def _serve(server: MockServer) -> None:
    await server.start()
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for signum in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(signum, stop.set)
    try:
        await stop.wait()
    finally:
        await server.stop()


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(name)s: %(message)s",
    )
    try:
        if args.check:
            report = asyncio.run(run_conformance(parse_endpoint(args.check)))
            for line in report.errors:
                print(f"FAIL {line}")
            print(
                f"{'ok' if report.ok else 'FAILED'}: {report.server_name}, "
                f"{report.signals_in_catalog} signals, {report.deltas} deltas, "
                f"{report.signal_blocks} signal blocks"
            )
            return 0 if report.ok else 1
        if args.write_sample:
            write_recording(generate_sample(args.duration, args.seed), args.write_sample)
            print(f"wrote {args.write_sample} ({args.write_sample.stat().st_size} bytes)")
            return 0
        if args.urdf:
            recording, report = import_urdf(args.urdf)
            for note in report.notes:
                print(f"note: {note}")
        elif args.recording:
            recording = read_recording(args.recording)
        else:
            recording = generate_sample(args.duration, args.seed)
        engine = ReplayEngine(recording, loop=not args.no_loop)
        if not args.paused:
            engine.play()
        asyncio.run(_serve(MockServer(engine, parse_endpoint(args.listen))))
    except (RecordingError, UrdfError, ValueError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
