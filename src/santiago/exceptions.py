"""Custom exceptions for the Santiago Task Manager."""


class SantiagoError(Exception):
    """Base exception for all Santiago errors."""


class TaskNotFoundError(SantiagoError):
    """Raised when a requested task does not exist in the repository."""

    def __init__(self, task_id: str) -> None:
        super().__init__(f"Task with id '{task_id}' was not found.")
        self.task_id = task_id


class DuplicateTaskError(SantiagoError):
    """Raised when trying to add a task whose id already exists."""

    def __init__(self, task_id: str) -> None:
        super().__init__(f"A task with id '{task_id}' already exists.")
        self.task_id = task_id
