#!/usr/bin/env python3
"""
run.py — Launch the Pipeline Inspection Dashboard
───────────────────────────────────────────────────
Usage:
    python3 run.py
    python3 run.py --host 0.0.0.0 --port 8000

This starts the FastAPI server with uvicorn.
Open http://<pi-ip>:8000 in a browser to view the dashboard.
"""

import argparse
import sys
from pathlib import Path

# Ensure project root is on path
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))


def main():
    # Load config for defaults
    from app.config import get as get_cfg
    c = get_cfg()
    srv = c.get("server", {})
    default_host = srv.get("host", "0.0.0.0")
    default_port = srv.get("port", 8000)

    parser = argparse.ArgumentParser(description="Pipeline Inspection Dashboard")
    parser.add_argument("--host", default=default_host, help=f"Bind address (default: {default_host})")
    parser.add_argument("--port", type=int, default=default_port, help=f"Port (default: {default_port})")
    parser.add_argument("--reload", action="store_true", help="Enable auto-reload for development")
    args = parser.parse_args()

    import uvicorn
    print(f"\n{'='*60}")
    print(f"  Pipeline Inspection Dashboard")
    print(f"  http://{args.host}:{args.port}")
    print(f"{'='*60}\n")

    uvicorn.run(
        "app.main:app",
        host=args.host,
        port=args.port,
        reload=args.reload,
        log_level="info",
    )


if __name__ == "__main__":
    main()
