#!/usr/bin/env python3
import argparse
import sys


def compute_checksum(body: str) -> int:
    checksum = 0
    for ch in body:
        checksum ^= ord(ch)
    return checksum


def parse_body(raw: str) -> tuple[str, str]:
    raw = raw.strip()
    if not raw:
        raise ValueError("message body is empty")
    start = "$"
    body = raw
    if raw[0] in ("$", "!"):
        start = raw[0]
        body = raw[1:]
    if "*" in body:
        body = body.split("*", 1)[0]
    return start, body


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate a NMEA0183 sentence with checksum.",
    )
    parser.add_argument(
        "body",
        nargs="?",
        help="Message body (e.g., IIMWV,045.0,R,10.2,N,A).",
    )
    parser.add_argument(
        "--crlf",
        action="store_true",
        help="Append CRLF line ending.",
    )
    args = parser.parse_args()

    raw = args.body
    if raw is None:
        raw = sys.stdin.read()

    start, body = parse_body(raw)
    checksum = compute_checksum(body)
    sentence = f"{start}{body}*{checksum:02X}"
    if args.crlf:
        sentence += "\r\n"
    print(sentence)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
