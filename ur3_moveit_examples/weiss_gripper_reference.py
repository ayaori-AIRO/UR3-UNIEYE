#!/usr/bin/env python3
"""Explicitly gated WEISS GRIPKIT XML-RPC reference test."""

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


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read the WEISS gripper state by default. With --execute, call "
            "Reference once and poll GetState until referencing completes."
        )
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--device-id", default=DEFAULT_DEVICE_ID)
    parser.add_argument("--connection-timeout", type=positive_float, default=3.0)
    parser.add_argument("--operation-timeout", type=positive_float, default=15.0)
    parser.add_argument("--poll-interval", type=positive_float, default=0.1)
    return parser.parse_args(argv)


def state_name(value) -> str:
    if isinstance(value, int):
        return STATE_NAMES.get(value, "UNKNOWN")
    return "INVALID_TYPE"


def print_state(proxy, device_id: str) -> tuple[object, object]:
    state = proxy.GetState(device_id)
    position = proxy.GetPos(device_id)
    print(format_result("GetState", state))
    print(f"  decoded_state: {state_name(state)}")
    print(format_result("GetPos", position))
    return state, position


def run(cli: argparse.Namespace) -> bool:
    transport = TimeoutTransport(cli.connection_timeout)
    with xmlrpc.client.ServerProxy(
        cli.url,
        transport=transport,
        allow_none=True,
        use_builtin_types=True,
    ) as proxy:
        print("State before reference:")
        initial_state, _ = print_state(proxy, cli.device_id)
        if not isinstance(initial_state, int):
            print("GetState returned an unexpected non-integer value.", file=sys.stderr)
            return False
        if initial_state == -1:
            print("Gripper reports COMMUNICATION_FAULT.", file=sys.stderr)
            return False
        if not cli.execute:
            print("\nREFERENCE NOT SENT: add --execute only after a safety check.")
            return True
        if initial_state != 0:
            print(
                "\nReference skipped: the gripper is already referenced "
                f"(state={state_name(initial_state)})."
            )
            return True

        print("\nCalling Reference exactly once. The gripper may move now.")
        result = proxy.Reference(cli.device_id)
        print(format_result("Reference", result))

        # The generated GRIPKIT URScript waits 0.2 seconds before polling.
        time.sleep(0.2)
        deadline = time.monotonic() + cli.operation_timeout
        previous_state = None
        while time.monotonic() < deadline:
            state = proxy.GetState(cli.device_id)
            position = proxy.GetPos(cli.device_id)
            if not isinstance(state, int):
                print(
                    f"GetState returned an unexpected value: {state!r}",
                    file=sys.stderr,
                )
                return False
            if state != previous_state:
                print(
                    f"Reference polling: state={state} ({state_name(state)}), "
                    f"position={position!r}"
                )
                previous_state = state
            if state == -1:
                print("Communication fault reported during reference.", file=sys.stderr)
                return False
            if state != 0:
                print("\nState after reference:")
                final_state, _ = print_state(proxy, cli.device_id)
                if final_state in (1, 2):
                    print("\nReference completed successfully.")
                    return True
                print(
                    "Reference left NOT_REFERENCED, but the resulting state is "
                    f"unexpected: {final_state!r} ({state_name(final_state)}).",
                    file=sys.stderr,
                )
                return False
            time.sleep(cli.poll_interval)

        print(
            "Reference timed out while the gripper remained NOT_REFERENCED.",
            file=sys.stderr,
        )
        return False


def main(args=None) -> None:
    cli = parse_args(sys.argv[1:] if args is None else args)
    print("WEISS GRIPKIT reference test")
    print(f"  URL: {cli.url}")
    print(f"  device_id: {cli.device_id!r}")
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
