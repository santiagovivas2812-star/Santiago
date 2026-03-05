"""Domain models for the Santiago Task Manager."""

from __future__ import annotations

import re
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum


class Priority(str, Enum):
    """Task priority levels."""

    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"


class Status(str, Enum):
    """Task lifecycle states."""

    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    DONE = "done"


_TITLE_MAX_LEN = 200
_TITLE_PATTERN = re.compile(r"^[\w\s.,!?'\-()#]+$", re.UNICODE)


@dataclass
class Task:
    """Represents a single task.

    Attributes:
        title: Short, human-readable name for the task (required).
        description: Optional longer description.
        priority: Urgency level (default: MEDIUM).
        status: Current lifecycle state (default: PENDING).
        id: Unique identifier (auto-generated UUID4).
        created_at: UTC timestamp set at creation time.
        updated_at: UTC timestamp updated on every modification.
    """

    title: str
    description: str | None = None
    priority: Priority = Priority.MEDIUM
    status: Status = Status.PENDING
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    created_at: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    updated_at: datetime = field(default_factory=lambda: datetime.now(timezone.utc))

    # ------------------------------------------------------------------
    # Validation helpers
    # ------------------------------------------------------------------

    def __post_init__(self) -> None:
        self._validate_title(self.title)
        if self.description is not None:
            self._validate_description(self.description)

    @staticmethod
    def _validate_title(title: str) -> None:
        if not title or not title.strip():
            raise ValueError("Task title must not be empty.")
        if len(title) > _TITLE_MAX_LEN:
            raise ValueError(
                f"Task title must be at most {_TITLE_MAX_LEN} characters long, "
                f"got {len(title)}."
            )
        if not _TITLE_PATTERN.match(title):
            raise ValueError(
                "Task title contains invalid characters. "
                "Only letters, digits, spaces and basic punctuation are allowed."
            )

    @staticmethod
    def _validate_description(description: str) -> None:
        if len(description) > 2000:
            raise ValueError(
                "Task description must be at most 2 000 characters long."
            )

    # ------------------------------------------------------------------
    # Mutation helpers (return a new Task to stay immutable-friendly)
    # ------------------------------------------------------------------

    def update(
        self,
        *,
        title: str | None = None,
        description: str | None = None,
        priority: Priority | None = None,
        status: Status | None = None,
    ) -> Task:
        """Return an updated copy of the task with a refreshed *updated_at*."""
        new_title = title if title is not None else self.title
        new_description = description if description is not None else self.description
        new_priority = priority if priority is not None else self.priority
        new_status = status if status is not None else self.status

        # Validate before constructing the new object.
        Task._validate_title(new_title)
        if new_description is not None:
            Task._validate_description(new_description)

        return Task(
            title=new_title,
            description=new_description,
            priority=new_priority,
            status=new_status,
            id=self.id,
            created_at=self.created_at,
            updated_at=datetime.now(timezone.utc),
        )

    def to_dict(self) -> dict:
        """Serialise to a plain dictionary (suitable for JSON output)."""
        return {
            "id": self.id,
            "title": self.title,
            "description": self.description,
            "priority": self.priority.value,
            "status": self.status.value,
            "created_at": self.created_at.isoformat(),
            "updated_at": self.updated_at.isoformat(),
        }

    @classmethod
    def from_dict(cls, data: dict) -> Task:
        """Deserialise from a plain dictionary."""
        return cls(
            title=data["title"],
            description=data.get("description"),
            priority=Priority(data.get("priority", Priority.MEDIUM.value)),
            status=Status(data.get("status", Status.PENDING.value)),
            id=data.get("id", str(uuid.uuid4())),
            created_at=datetime.fromisoformat(data["created_at"])
            if "created_at" in data
            else datetime.now(timezone.utc),
            updated_at=datetime.fromisoformat(data["updated_at"])
            if "updated_at" in data
            else datetime.now(timezone.utc),
        )
