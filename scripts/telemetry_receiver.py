#!/usr/bin/env python3
"""
Faketec telemetry receiver.

Listens for Reticulum announces on aspect "faketec.telemetry" and prints a
timestamped line for every one received. Each announce carries a small
ASCII key=value payload in the announce's app_data field, populated by the
node firmware when BAKED_TELEMETRY_ENABLE is compiled in.

Run on any host that has a Reticulum interface reachable from the remote
Faketec nodes (RNode USB dongle, TCP link to another transport node, etc.).
Reticulum will use whatever interfaces are configured in
~/.reticulum/config, so you do not need to configure anything here.

Setup:
    pip install rns

Usage:
    python scripts/telemetry_receiver.py [--config /path/to/reticulum/config]

Output format (one line per announce):
    2026-04-04T21:30:00  <dest_hash>  bat=3987 up=7234 hpf=54321 ro=1 rssi=-95 nf=-105
"""

import argparse
import datetime
import sys

import RNS


APP_NAME = "faketec"
ASPECTS = "telemetry"


class TelemetryHandler:
    """RNS announce handler that filters on the faketec.telemetry aspect."""

    # The aspect_filter attribute is how RNS decides which announces to
    # deliver to this handler — it matches by dotted app.aspects string.
    aspect_filter = f"{APP_NAME}.{ASPECTS}"

    def received_announce(self, destination_hash, announced_identity, app_data):
        ts = datetime.datetime.now().replace(microsecond=0).isoformat()
        hash_hex = RNS.prettyhexrep(destination_hash)
        if app_data is None:
            payload = "(no app_data)"
        else:
            try:
                payload = app_data.decode("ascii", errors="replace")
            except Exception:
                payload = repr(app_data)
        # Normalize the ; separators to spaces for a cleaner single-line log.
        payload_display = payload.replace(";", " ")
        print(f"{ts}  {hash_hex}  {payload_display}", flush=True)


def main():
    parser = argparse.ArgumentParser(
        description="Receive Faketec telemetry announces from the Reticulum mesh.",
    )
    parser.add_argument(
        "--config",
        default=None,
        help="Path to a Reticulum config directory (defaults to ~/.reticulum).",
    )
    parser.add_argument(
        "--verbosity",
        type=int,
        default=0,
        help="RNS log verbosity 0-7 (default 0).",
    )
    args = parser.parse_args()

    reticulum = RNS.Reticulum(configdir=args.config, loglevel=args.verbosity)

    handler = TelemetryHandler()
    RNS.Transport.register_announce_handler(handler)

    print(
        f"Listening for announces on aspect {handler.aspect_filter!r}...",
        file=sys.stderr,
        flush=True,
    )
    print("Press Ctrl+C to stop.", file=sys.stderr, flush=True)

    try:
        while True:
            # Reticulum runs its own threads; this main thread just parks.
            import time
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down.", file=sys.stderr)


if __name__ == "__main__":
    main()
