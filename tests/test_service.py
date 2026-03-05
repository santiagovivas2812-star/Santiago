"""Tests for TaskService."""

from __future__ import annotations

import pytest

from santiago.exceptions import TaskNotFoundError
from santiago.models import Priority, Status, Task
from santiago.repository import TaskRepository
from santiago.service import TaskService


@pytest.fixture()
def svc() -> TaskService:
    return TaskService(TaskRepository())


# ---------------------------------------------------------------------------
# create_task
# ---------------------------------------------------------------------------


def test_create_task_returns_task(svc):
    task = svc.create_task("My task")
    assert isinstance(task, Task)
    assert task.title == "My task"
    assert task.status == Status.PENDING


def test_create_task_with_priority(svc):
    task = svc.create_task("Urgent", priority=Priority.HIGH)
    assert task.priority == Priority.HIGH


def test_create_task_invalid_title(svc):
    with pytest.raises(ValueError):
        svc.create_task("")


# ---------------------------------------------------------------------------
# update_task
# ---------------------------------------------------------------------------


def test_update_task_changes_fields(svc):
    task = svc.create_task("Original")
    updated = svc.update_task(task.id, title="Updated", priority=Priority.LOW)
    assert updated.title == "Updated"
    assert updated.priority == Priority.LOW


def test_update_task_not_found(svc):
    with pytest.raises(TaskNotFoundError):
        svc.update_task("no-such-id", status=Status.DONE)


# ---------------------------------------------------------------------------
# complete_task
# ---------------------------------------------------------------------------


def test_complete_task(svc):
    task = svc.create_task("Finish me")
    completed = svc.complete_task(task.id)
    assert completed.status == Status.DONE


def test_complete_task_not_found(svc):
    with pytest.raises(TaskNotFoundError):
        svc.complete_task("missing")


# ---------------------------------------------------------------------------
# delete_task
# ---------------------------------------------------------------------------


def test_delete_task(svc):
    task = svc.create_task("Delete me")
    svc.delete_task(task.id)
    with pytest.raises(TaskNotFoundError):
        svc.get_task(task.id)


def test_delete_task_not_found(svc):
    with pytest.raises(TaskNotFoundError):
        svc.delete_task("ghost")


# ---------------------------------------------------------------------------
# list_tasks / pending_count
# ---------------------------------------------------------------------------


def test_list_tasks_empty(svc):
    assert svc.list_tasks() == []


def test_list_tasks_filter(svc):
    svc.create_task("A", priority=Priority.HIGH)
    svc.create_task("B", priority=Priority.LOW)
    highs = svc.list_tasks(priority=Priority.HIGH)
    assert len(highs) == 1
    assert highs[0].title == "A"


def test_pending_count(svc):
    svc.create_task("Task 1")
    t2 = svc.create_task("Task 2")
    svc.complete_task(t2.id)
    assert svc.pending_count() == 1
