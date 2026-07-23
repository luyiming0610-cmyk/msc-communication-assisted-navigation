#!/usr/bin/env python3
"""Shared, dependency-free framing for the e-puck TCP bridge.

The module intentionally supports Python 3.6 because the Pi-puck Foxy image
uses that interpreter.  Each newline-delimited JSON envelope contains a
canonical payload and a CRC32 calculated over that payload.
"""

import hmac
import json
import zlib


PROTOCOL_VERSION = "epuck_bridge_v1"
MAX_LINE_BYTES = 1024 * 1024


class ProtocolError(ValueError):
    pass


def _canonical_payload(payload):
    return json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")


def encode_message(payload):
    if not isinstance(payload, dict):
        raise ProtocolError("payload must be a JSON object")

    body = dict(payload)
    body.setdefault("protocol", PROTOCOL_VERSION)
    canonical = _canonical_payload(body)
    crc32 = "{:08x}".format(zlib.crc32(canonical) & 0xFFFFFFFF)
    envelope = {"crc32": crc32, "payload": body}
    encoded = json.dumps(
        envelope,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8") + b"\n"

    if len(encoded) > MAX_LINE_BYTES:
        raise ProtocolError("encoded message exceeds size limit")
    return encoded


def decode_message_line(line):
    if isinstance(line, str):
        line = line.encode("utf-8")
    if not isinstance(line, (bytes, bytearray)):
        raise ProtocolError("line must be bytes or text")
    if len(line) > MAX_LINE_BYTES:
        raise ProtocolError("message exceeds size limit")

    try:
        envelope = json.loads(bytes(line).decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as exc:
        raise ProtocolError("invalid JSON: {}".format(exc))

    if not isinstance(envelope, dict):
        raise ProtocolError("envelope must be a JSON object")
    payload = envelope.get("payload")
    received_crc = envelope.get("crc32")
    if not isinstance(payload, dict) or not isinstance(received_crc, str):
        raise ProtocolError("envelope is missing payload or crc32")
    if payload.get("protocol") != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol version")

    canonical = _canonical_payload(payload)
    expected_crc = "{:08x}".format(zlib.crc32(canonical) & 0xFFFFFFFF)
    if not hmac.compare_digest(received_crc.lower(), expected_crc):
        raise ProtocolError("CRC32 mismatch")
    return payload


class LineBuffer(object):
    def __init__(self, max_line_bytes=MAX_LINE_BYTES):
        self._buffer = bytearray()
        self._max_line_bytes = int(max_line_bytes)

    def feed(self, data):
        if not isinstance(data, (bytes, bytearray)):
            raise ProtocolError("socket data must be bytes")
        self._buffer.extend(data)
        lines = []

        while True:
            newline = self._buffer.find(b"\n")
            if newline < 0:
                if len(self._buffer) > self._max_line_bytes:
                    self._buffer = bytearray()
                    raise ProtocolError("unterminated message exceeds size limit")
                break

            line = bytes(self._buffer[:newline])
            del self._buffer[:newline + 1]
            if not line:
                continue
            if len(line) > self._max_line_bytes:
                raise ProtocolError("message exceeds size limit")
            lines.append(line)

        return lines

