#!/usr/bin/env python3
"""Read-only XML-RPC connection test for a WEISS GRIPKIT gripper."""

import argparse
import math
import socket
import sys
import xmlrpc.client


DEFAULT_URL = "http://192.168.1.11:44221/RPC2"
DEFAULT_DEVICE_ID = "IEG 55-020 SN:000234"
STATE_NAMES = {
    -1: "COMMUNICATION_FAULT",
    0: "NOT_REFERENCED",
    1: "IDLE",
    2: "RELEASED",
    4: "NO_PART",
    8: "HOLDING",
}


class TimeoutTransport(xmlrpc.client.Transport):
    """XML-RPC HTTP transport with an explicit socket timeout."""

    def __init__(self, timeout: float):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self.timeout
        return connection


def positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Call only GetState and GetPos on the WEISS GRIPKIT XML-RPC "
            "daemon. This test does not command gripper motion."
        )
    )
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--device-id", default=DEFAULT_DEVICE_ID)
    parser.add_argument("--timeout", type=positive_float, default=3.0)
    return parser.parse_args(argv)


def format_result(method: str, value) -> str:
    return (
        f"{method} response:\n"
        f"  python_type: {type(value).__module__}.{type(value).__qualname__}\n"
        f"  repr: {value!r}\n"
        f"  str: {value}"
    )


def main(args=None) -> None:
    cli = parse_args(sys.argv[1:] if args is None else args)
    print("WEISS GRIPKIT read-only XML-RPC test")
    print(f"  URL: {cli.url}")
    print(f"  device_id: {cli.device_id!r}")
    print(f"  timeout: {cli.timeout:.3f} s")
    print("  methods: GetState, GetPos (no motion commands)\n")

    transport = TimeoutTransport(cli.timeout)
    try:
        with xmlrpc.client.ServerProxy(
            cli.url,
            transport=transport,
            allow_none=True,
            use_builtin_types=True,
        ) as proxy:
            state = proxy.GetState(cli.device_id)
            print(format_result("GetState", state))
            if isinstance(state, int):
                print(f"  decoded_state: {STATE_NAMES.get(state, 'UNKNOWN')}")
            position = proxy.GetPos(cli.device_id)
            print(format_result("GetPos", position))
    except xmlrpc.client.Fault as exc:
        print(
            "XML-RPC fault:\n"
            f"  faultCode: {exc.faultCode}\n"
            f"  faultString: {exc.faultString}",
            file=sys.stderr,
        )
        raise SystemExit(2) from exc
    except xmlrpc.client.ProtocolError as exc:
        print(
            "XML-RPC HTTP protocol error:\n"
            f"  url: {exc.url}\n"
            f"  http_status: {exc.errcode}\n"
            f"  message: {exc.errmsg}",
            file=sys.stderr,
        )
        raise SystemExit(3) from exc
    except (socket.timeout, TimeoutError) as exc:
        print(
            f"XML-RPC connection timed out after {cli.timeout:.3f} s: {exc}",
            file=sys.stderr,
        )
        raise SystemExit(4) from exc
    except (ConnectionError, OSError) as exc:
        print(
            f"XML-RPC connection failed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        raise SystemExit(5) from exc
    except xmlrpc.client.Error as exc:
        print(
            f"XML-RPC client error: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        raise SystemExit(6) from exc

    print("\nRead-only XML-RPC test completed successfully.")


if __name__ == "__main__":
    main()
