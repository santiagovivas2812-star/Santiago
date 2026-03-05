"""Business-logic layer for the Santiago Task Manager."""

from __future__ import annotations

from .models import Priority, Status, Task
from .repository import TaskRepository


class TaskService:
    """High-level operations over a :class:`TaskRepository`.

    All validation that spans multiple domain rules lives here, keeping
    the repository thin (data access only) and the model focused on
    per-object invariants.

    Args:
        repository: The repository instance to delegate storage to.
    """

    def __init__(self, repository: TaskRepository) -> None:
        self._repo = repository

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def create_task(
        self,
        title: str,
        *,
        description: str | None = None,
        priority: Priority = Priority.MEDIUM,
    ) -> Task:
        """Create and persist a new task.

        Args:
            title: Human-readable task name.
            description: Optional longer explanation.
            priority: Urgency level (default: MEDIUM).

        Returns:
            The newly created :class:`Task`.

        Raises:
            ValueError: If *title* is blank or contains invalid characters.
        """
        task = Task(title=title, description=description, priority=priority)
        return self._repo.add(task)

    def update_task(
        self,
        task_id: str,
        *,
        title: str | None = None,
        description: str | None = None,
        priority: Priority | None = None,
        status: Status | None = None,
    ) -> Task:
        """Update an existing task.

        Only the fields that are explicitly passed (non-None) are changed.

        Args:
            task_id: The id of the task to update.
            title: New title, if changing.
            description: New description, if changing.
            priority: New priority, if changing.
            status: New status, if changing.

        Returns:
            The updated :class:`Task`.

        Raises:
            TaskNotFoundError: If *task_id* does not exist.
            ValueError: If any supplied value fails model-level validation.
        """
        existing = self._repo.get(task_id)
        updated = existing.update(
            title=title,
            description=description,
            priority=priority,
            status=status,
        )
        return self._repo.update(updated)

    def complete_task(self, task_id: str) -> Task:
        """Mark a task as *done*.

        Args:
            task_id: The id of the task to mark complete.

        Returns:
            The updated :class:`Task`.

        Raises:
            TaskNotFoundError: If *task_id* does not exist.
        """
        return self.update_task(task_id, status=Status.DONE)

    def delete_task(self, task_id: str) -> None:
        """Permanently remove a task.

        Args:
            task_id: The id of the task to delete.

        Raises:
            TaskNotFoundError: If *task_id* does not exist.
        """
        self._repo.delete(task_id)

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def get_task(self, task_id: str) -> Task:
        """Return a single task by id.

        Raises:
            TaskNotFoundError: If *task_id* does not exist.
        """
        return self._repo.get(task_id)

    def list_tasks(
        self,
        *,
        status: Status | None = None,
        priority: Priority | None = None,
    ) -> list[Task]:
        """Return tasks, optionally filtered.

        Args:
            status: Keep only tasks with this status.
            priority: Keep only tasks with this priority.

        Returns:
            Sorted list of matching :class:`Task` objects (oldest first).
        """
        return self._repo.list_all(status=status, priority=priority)

    def pending_count(self) -> int:
        """Return the number of tasks that are not yet *done*."""
        return sum(
            1 for t in self._repo.list_all() if t.status != Status.DONE
        )
