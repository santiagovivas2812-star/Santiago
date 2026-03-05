"""Tests for Task model validation and behaviour."""

from __future__ import annotations

import pytest

from santiago.models import Priority, Status, Task

# ---------------------------------------------------------------------------
# Valid construction
# ---------------------------------------------------------------------------


def test_task_defaults():
    task = Task(title="Buy groceries")
    assert task.title == "Buy groceries"
    assert task.priority == Priority.MEDIUM
    assert task.status == Status.PENDING
    assert task.description is None
    assert task.id  # non-empty UUID string


def test_task_with_all_fields():
    task = Task(
        title="Deploy app",
        description="Push to production.",
        priority=Priority.HIGH,
        status=Status.IN_PROGRESS,
    )
    assert task.priority == Priority.HIGH
    assert task.status == Status.IN_PROGRESS


def test_task_id_is_unique():
    t1 = Task(title="Task A")
    t2 = Task(title="Task B")
    assert t1.id != t2.id


# ---------------------------------------------------------------------------
# Title validation
# ---------------------------------------------------------------------------


def test_empty_title_raises():
    with pytest.raises(ValueError, match="empty"):
        Task(title="")


def test_whitespace_only_title_raises():
    with pytest.raises(ValueError, match="empty"):
        Task(title="   ")


def test_title_too_long_raises():
    with pytest.raises(ValueError, match="200 characters"):
        Task(title="x" * 201)


def test_title_max_length_ok():
    task = Task(title="a" * 200)
    assert len(task.title) == 200


def test_title_invalid_chars_raises():
    with pytest.raises(ValueError, match="invalid characters"):
        Task(title="Bad<Script>")


def test_title_valid_punctuation():
    task = Task(title="Fix bug #1 - it's urgent!")
    assert task.title == "Fix bug #1 - it's urgent!"


# ---------------------------------------------------------------------------
# Description validation
# ---------------------------------------------------------------------------


def test_description_too_long_raises():
    with pytest.raises(ValueError, match="2 000 characters"):
        Task(title="Valid", description="d" * 2001)


def test_description_max_length_ok():
    task = Task(title="Valid", description="d" * 2000)
    assert len(task.description) == 2000


# ---------------------------------------------------------------------------
# update() method
# ---------------------------------------------------------------------------


def test_update_title():
    task = Task(title="Old title")
    updated = task.update(title="New title")
    assert updated.title == "New title"
    assert updated.id == task.id
    assert updated.updated_at >= task.updated_at


def test_update_status():
    task = Task(title="Work item")
    updated = task.update(status=Status.DONE)
    assert updated.status == Status.DONE
    # original is unchanged
    assert task.status == Status.PENDING


def test_update_with_invalid_title_raises():
    task = Task(title="Original")
    with pytest.raises(ValueError):
        task.update(title="")


def test_update_preserves_unchanged_fields():
    task = Task(title="Original", priority=Priority.HIGH, description="desc")
    updated = task.update(status=Status.IN_PROGRESS)
    assert updated.title == "Original"
    assert updated.priority == Priority.HIGH
    assert updated.description == "desc"


# ---------------------------------------------------------------------------
# Serialisation round-trip
# ---------------------------------------------------------------------------


def test_to_dict_keys():
    task = Task(title="Serialise me")
    d = task.to_dict()
    assert set(d.keys()) == {
        "id", "title", "description", "priority", "status",
        "created_at", "updated_at",
    }


def test_from_dict_round_trip():
    task = Task(title="Round trip", priority=Priority.LOW, description="Hi")
    restored = Task.from_dict(task.to_dict())
    assert restored.id == task.id
    assert restored.title == task.title
    assert restored.description == task.description
    assert restored.priority == task.priority
    assert restored.status == task.status
