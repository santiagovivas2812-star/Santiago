"""Command-line interface for the Santiago Task Manager."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .exceptions import SantiagoError
from .models import Priority, Status
from .repository import TaskRepository
from .service import TaskService

_DEFAULT_DB = Path.home() / ".santiago" / "tasks.json"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="santiago",
        description="Santiago Task Manager — manage your tasks from the command line.",
    )
    parser.add_argument(
        "--db",
        metavar="PATH",
        type=Path,
        default=_DEFAULT_DB,
        help="Path to the JSON storage file (default: %(default)s).",
    )
    parser.add_argument(
        "--format",
        choices=["table", "json"],
        default="table",
        help="Output format (default: table).",
    )

    sub = parser.add_subparsers(dest="command", required=True)

    # add
    add_p = sub.add_parser("add", help="Create a new task.")
    add_p.add_argument("title", help="Task title.")
    add_p.add_argument("-d", "--description", help="Optional description.")
    add_p.add_argument(
        "-p",
        "--priority",
        choices=[p.value for p in Priority],
        default=Priority.MEDIUM.value,
    )

    # list
    list_p = sub.add_parser("list", help="List tasks.")
    list_p.add_argument(
        "--status",
        choices=[s.value for s in Status],
        default=None,
    )
    list_p.add_argument(
        "--priority",
        choices=[p.value for p in Priority],
        default=None,
    )

    # done
    done_p = sub.add_parser("done", help="Mark a task as done.")
    done_p.add_argument("id", help="Task id.")

    # delete
    del_p = sub.add_parser("delete", help="Delete a task.")
    del_p.add_argument("id", help="Task id.")

    return parser


def _ensure_db_dir(db_path: Path) -> None:
    db_path.parent.mkdir(parents=True, exist_ok=True)


def _print_table(tasks: list) -> None:
    if not tasks:
        print("No tasks found.")
        return
    fmt = "{:<36}  {:<30}  {:<8}  {:<12}"
    print(fmt.format("ID", "TITLE", "PRIORITY", "STATUS"))
    print("-" * 90)
    for t in tasks:
        title_display = t.title[:28] + ".." if len(t.title) > 30 else t.title
        print(fmt.format(t.id, title_display, t.priority.value, t.status.value))


def main(argv: list | None = None) -> int:
    """Entry point.  Returns an exit code (0 = success)."""
    parser = _build_parser()
    args = parser.parse_args(argv)

    try:
        _ensure_db_dir(args.db)
        repo = TaskRepository(storage_path=args.db)
        svc = TaskService(repo)

        if args.command == "add":
            task = svc.create_task(
                args.title,
                description=args.description,
                priority=Priority(args.priority),
            )
            if args.format == "json":
                print(json.dumps(task.to_dict(), indent=2))
            else:
                print(f"✓ Task created: {task.id}")

        elif args.command == "list":
            status_filter = Status(args.status) if args.status else None
            priority_filter = Priority(args.priority) if args.priority else None
            tasks = svc.list_tasks(status=status_filter, priority=priority_filter)
            if args.format == "json":
                print(json.dumps([t.to_dict() for t in tasks], indent=2))
            else:
                _print_table(tasks)

        elif args.command == "done":
            task = svc.complete_task(args.id)
            if args.format == "json":
                print(json.dumps(task.to_dict(), indent=2))
            else:
                print(f"✓ Task '{task.title}' marked as done.")

        elif args.command == "delete":
            svc.delete_task(args.id)
            if args.format != "json":
                print(f"✓ Task {args.id} deleted.")

    except SantiagoError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    except ValueError as exc:
        print(f"Validation error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
