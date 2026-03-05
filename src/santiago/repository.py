"""In-memory task repository with optional JSON persistence."""

from __future__ import annotations

import json
import os
from pathlib import Path

from .exceptions import DuplicateTaskError, TaskNotFoundError
from .models import Priority, Status, Task


class TaskRepository:
    """Stores and retrieves :class:`Task` objects.

    By default tasks live only in memory.  Pass a *storage_path* to enable
    automatic JSON persistence — the file is read on initialisation and
    written on every mutating operation.

    Args:
        storage_path: Optional path to a JSON file used for persistence.
            The parent directory must already exist.

    Raises:
        ValueError: If *storage_path* is provided but its parent directory
            does not exist.
        json.JSONDecodeError: If the storage file exists but contains
            invalid JSON.
    """

    def __init__(self, storage_path: Path | None = None) -> None:
        self._tasks: dict[str, Task] = {}
        self._storage_path = storage_path

        if storage_path is not None:
            parent = storage_path.parent
            if not parent.exists():
                raise ValueError(
                    f"Storage directory '{parent}' does not exist. "
                    "Please create it before initialising the repository."
                )
            if storage_path.exists():
                self._load()

    # ------------------------------------------------------------------
    # CRUD
    # ------------------------------------------------------------------

    def add(self, task: Task) -> Task:
        """Persist a new *task*.

        Raises:
            DuplicateTaskError: If a task with the same *id* already exists.
        """
        if task.id in self._tasks:
            raise DuplicateTaskError(task.id)
        self._tasks[task.id] = task
        self._save()
        return task

    def get(self, task_id: str) -> Task:
        """Return the task identified by *task_id*.

        Raises:
            TaskNotFoundError: If no task with *task_id* exists.
        """
        if task_id not in self._tasks:
            raise TaskNotFoundError(task_id)
        return self._tasks[task_id]

    def list_all(
        self,
        *,
        status: Status | None = None,
        priority: Priority | None = None,
    ) -> list[Task]:
        """Return tasks, optionally filtered by *status* and/or *priority*.

        The returned list is sorted by *created_at* (oldest first).
        """
        tasks = self._tasks.values()
        if status is not None:
            tasks = (t for t in tasks if t.status == status)
        if priority is not None:
            tasks = (t for t in tasks if t.priority == priority)
        return sorted(tasks, key=lambda t: t.created_at)

    def update(self, task: Task) -> Task:
        """Replace the stored copy of *task* (matched by *id*).

        Raises:
            TaskNotFoundError: If *task.id* is not present in the store.
        """
        if task.id not in self._tasks:
            raise TaskNotFoundError(task.id)
        self._tasks[task.id] = task
        self._save()
        return task

    def delete(self, task_id: str) -> None:
        """Remove the task identified by *task_id*.

        Raises:
            TaskNotFoundError: If no task with *task_id* exists.
        """
        if task_id not in self._tasks:
            raise TaskNotFoundError(task_id)
        del self._tasks[task_id]
        self._save()

    def count(self) -> int:
        """Return the total number of stored tasks."""
        return len(self._tasks)

    # ------------------------------------------------------------------
    # Persistence helpers
    # ------------------------------------------------------------------

    def _save(self) -> None:
        if self._storage_path is None:
            return
        data = [task.to_dict() for task in self._tasks.values()]
        tmp_path = self._storage_path.with_suffix(".tmp")
        tmp_path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
        os.replace(tmp_path, self._storage_path)

    def _load(self) -> None:
        raw = self._storage_path.read_text(encoding="utf-8")
        data: list = json.loads(raw)
        for item in data:
            task = Task.from_dict(item)
            self._tasks[task.id] = task
