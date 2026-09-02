#!/usr/bin/env python3
"""Explicitly gated WEISS GRIPKIT XML-RPC release test."""

import argparse
import socket
import sys
import time
import xmlrpc.client

from ur3_moveit_examples.weiss_gripper_read_state import (
    DEFAULT_DEVICE_ID,
    DEFAULT_URL,
    STATE_NAMES,
    TimeoutTransport,
    format_result,
    positive_float,
)


def non_negative_int(value: str) -> int:
    parsed = int(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read the WEISS gripper state by default. With --execute, call "
            "Release once and poll GetState until RELEASED."
        )
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--index", type=non_negative_int, default=0)
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--device-id", default=DEFAULT_DEVICE_ID)
    parser.add_argument("--connection-timeout", type=positive_float, default=3.0)
    parser.add_argument("--operation-timeout", type=positive_float, default=10.0)
    parser.add_argument("--poll-interval", type=positive_float, default=0.1)
    return parser.parse_args(argv)


def state_name(value) -> str:
    if isinstance(value, int):
        return STATE_NAMES.get(value, "UNKNOWN")
    return "INVALID_TYPE"


def read_status(proxy, device_id: str) -> tuple[object, object]:
    return proxy.GetState(device_id), proxy.GetPos(device_id)


def run(cli: argparse.Namespace) -> bool:
    transport = TimeoutTransport(cli.connection_timeout)
    with xmlrpc.client.ServerProxy(
        cli.url,
        transport=transport,
        allow_none=True,
        use_builtin_types=True,
    ) as proxy:
        state, position = read_status(proxy, cli.device_id)
        print(
            f"State before release: {state!r} ({state_name(state)}), "
            f"position={position!r}"
        )
        if not isinstance(state, int):
            print("GetState returned an unexpected non-integer value.", file=sys.stderr)
            return False
        if state == -1:
            print("Gripper reports COMMUNICATION_FAULT.", file=sys.stderr)
            return False
        if state == 0:
            print(
                "Release blocked: the gripper is NOT_REFERENCED. Run the "
                "reference test first.",
                file=sys.stderr,
            )
            return False
        if not cli.execute:
            print("RELEASE NOT SENT: add --execute only after a safety check.")
            return True
        if state == 2:
            print("Release skipped: the gripper already reports RELEASED.")
            return True

        print(
            f"Calling Release exactly once with grip index {cli.index}. "
            "The gripper may move now."
        )
        result = proxy.Release(cli.device_id, cli.index)
        print(format_result("Release", result))

        deadline = time.monotonic() + cli.operation_timeout
        previous = None
        while time.monotonic() < deadline:
            state, position = read_status(proxy, cli.device_id)
            if not isinstance(state, int):
                print(f"Unexpected GetState value: {state!r}", file=sys.stderr)
                return False
            snapshot = (state, position)
            if snapshot != previous:
                print(
                    f"Release polling: state={state} ({state_name(state)}), "
                    f"position={position!r}"
                )
                previous = snapshot
            if state == -1:
                print("Communication fault reported during release.", file=sys.stderr)
                return False
            if state == 0:
                print("Gripper lost its reference during release.", file=sys.stderr)
                return False
            if state == 2:
                print("Release completed successfully.")
                return True
            time.sleep(cli.poll_interval)

        print(
            f"Release timed out before RELEASED; last state={state!r} "
            f"({state_name(state)}), position={position!r}.",
            file=sys.stderr,
        )
        return False


def main(args=None) -> None:
    cli = parse_args(sys.argv[1:] if args is None else args)
    print("WEISS GRIPKIT release test")
    print(f"  URL: {cli.url}")
    print(f"  device_id: {cli.device_id!r}")
    print(f"  grip_index: {cli.index}")
    print(f"  execute: {cli.execute}")
    try:
        success = run(cli)
    except xmlrpc.client.Fault as exc:
        print(
            f"XML-RPC fault: code={exc.faultCode}, message={exc.faultString}",
            file=sys.stderr,
        )
        success = False
    except xmlrpc.client.ProtocolError as exc:
        print(
            f"XML-RPC HTTP error: status={exc.errcode}, message={exc.errmsg}",
            file=sys.stderr,
        )
        success = False
    except (socket.timeout, TimeoutError) as exc:
        print(f"XML-RPC connection timed out: {exc}", file=sys.stderr)
        success = False
    except (ConnectionError, OSError, xmlrpc.client.Error) as exc:
        print(
            f"XML-RPC communication failed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        success = False
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
