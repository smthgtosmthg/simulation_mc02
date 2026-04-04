#!/usr/bin/env python3
"""
Active Inference Dashboard — Backend Server
============================================
Serves the dashboard UI and REST API reading simulation JSON outputs.

Usage:
    python3 server.py [--port 8060]
    Open http://localhost:8060
"""

import argparse
import json
import os
from http.server import HTTPServer, SimpleHTTPRequestHandler

DASHBOARD_DIR = os.path.dirname(os.path.abspath(__file__))
STATE_PATH = "/tmp/aif_state.json"
HISTORY_PATH = "/tmp/aif_history.json"


class DashboardHandler(SimpleHTTPRequestHandler):
    """Serves static files from dashboard_aif/ and JSON API endpoints."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DASHBOARD_DIR, **kwargs)

    def do_GET(self):
        if self.path == "/api/state":
            self._serve_json(STATE_PATH)
        elif self.path == "/api/history":
            self._serve_json(HISTORY_PATH)
        else:
            super().do_GET()

    def _serve_json(self, filepath: str):
        try:
            with open(filepath, "r") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data.encode())
        except FileNotFoundError:
            self.send_response(404)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"error":"data not available yet"}')

    def log_message(self, format, *args):
        # suppress per-request log noise; only log errors
        if args and "200" not in str(args[0]):
            super().log_message(format, *args)


def main():
    parser = argparse.ArgumentParser(description="AIF Dashboard Server")
    parser.add_argument("--port", type=int, default=8060)
    args = parser.parse_args()

    server = HTTPServer(("0.0.0.0", args.port), DashboardHandler)
    print(f"[Dashboard] http://localhost:{args.port}")
    print(f"[Dashboard] Reading {STATE_PATH}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[Dashboard] Stopped.")
        server.server_close()


if __name__ == "__main__":
    main()
