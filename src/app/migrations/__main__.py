# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""CLI entry point for database migrations.

Usage:
    python -m app.migrations migrate              # Apply all pending
    python -m app.migrations migrate --to 2       # Migrate up to version 2
    python -m app.migrations rollback 1           # Rollback to version 1
    python -m app.migrations status               # Show migration status
"""

import argparse
import asyncio
import sys

from loguru import logger


async def _run(args: argparse.Namespace) -> int:
    """Execute the requested migration command."""
    from app.database import engine
    from app.migrations.migrator import MigrationManager

    manager = MigrationManager(engine)

    if args.command == "migrate":
        target = args.to if hasattr(args, "to") else None
        applied = await manager.migrate(target=target)
        version = await manager.get_version()
        print(f"Schema version: {version} ({applied} migration(s) applied)")
        return 0

    elif args.command == "rollback":
        target = args.version
        rolled = await manager.rollback(target)
        version = await manager.get_version()
        print(f"Schema version: {version} ({rolled} migration(s) rolled back)")
        return 0

    elif args.command == "status":
        entries = await manager.status()
        version = await manager.get_version()
        print(f"Current schema version: {version}\n")
        print(f"{'Ver':>4}  {'Status':<10}  {'Applied At':<25}  Description")
        print("-" * 75)
        for e in entries:
            status = "applied" if e["applied"] else "pending"
            ts = e["applied_at"] or ""
            print(f"{e['version']:>4}  {status:<10}  {ts:<25}  {e['description']}")
        if not entries:
            print("  (no migrations found)")
        return 0

    else:
        print(f"Unknown command: {args.command}", file=sys.stderr)
        return 1


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="python -m app.migrations",
        description="Tritium-SC database migration tool",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    migrate_p = sub.add_parser("migrate", help="Apply pending migrations")
    migrate_p.add_argument("--to", type=int, default=None, help="Target version")

    rollback_p = sub.add_parser("rollback", help="Rollback to a specific version")
    rollback_p.add_argument("version", type=int, help="Target version to rollback to")

    sub.add_parser("status", help="Show migration status")

    args = parser.parse_args()

    # Configure minimal logging
    logger.remove()
    logger.add(sys.stderr, level="INFO", format="{message}")

    rc = asyncio.run(_run(args))
    sys.exit(rc)


if __name__ == "__main__":
    main()
