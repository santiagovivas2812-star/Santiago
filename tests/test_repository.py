"""Tests for TaskRepository."""

from __future__ import annotations

import json

import pytest

from santiago.exceptions import DuplicateTaskError, TaskNotFoundError
from santiago.models import Priority, Status, Task
from santiago.repository import TaskRepository


@pytest.fixture()
def repo() -> TaskRepository:
    return TaskRepository()


@pytest.fixture()
def sample_task() -> Task:
    return Task(title="Sample task")


# ---------------------------------------------------------------------------
# add / get
# ---------------------------------------------------------------------------


def test_add_and_get(repo, sample_task):
    repo.add(sample_task)
    fetched = repo.get(sample_task.id)
    assert fetched.id == sample_task.id


def test_add_duplicate_raises(repo, sample_task):
    repo.add(sample_task)
    with pytest.raises(DuplicateTaskError):
        repo.add(sample_task)


def test_get_missing_raises(repo):
    with pytest.raises(TaskNotFoundError):
        repo.get("non-existent-id")


# ---------------------------------------------------------------------------
# list_all
# ---------------------------------------------------------------------------


def test_list_all_empty(repo):
    assert repo.list_all() == []


def test_list_all_returns_all(repo):
    for i in range(3):
        repo.add(Task(title=f"Task {i}"))
    assert len(repo.list_all()) == 3


def test_list_all_filter_by_status(repo):
    t1 = Task(title="Pending task")
    t2 = Task(title="Done task", status=Status.DONE)
    repo.add(t1)
    repo.add(t2)
    pending = repo.list_all(status=Status.PENDING)
    assert len(pending) == 1
    assert pending[0].id == t1.id


def test_list_all_filter_by_priority(repo):
    repo.add(Task(title="Low", priority=Priority.LOW))
    repo.add(Task(title="High", priority=Priority.HIGH))
    highs = repo.list_all(priority=Priority.HIGH)
    assert len(highs) == 1
    assert highs[0].priority == Priority.HIGH


def test_list_all_sorted_by_created_at(repo):
    for i in range(5):
        repo.add(Task(title=f"Task {i}"))
    tasks = repo.list_all()
    assert tasks == sorted(tasks, key=lambda t: t.created_at)


# ---------------------------------------------------------------------------
# update
# ---------------------------------------------------------------------------


def test_update_existing(repo, sample_task):
    repo.add(sample_task)
    updated = sample_task.update(status=Status.DONE)
    result = repo.update(updated)
    assert result.status == Status.DONE
    assert repo.get(sample_task.id).status == Status.DONE


def test_update_missing_raises(repo, sample_task):
    with pytest.raises(TaskNotFoundError):
        repo.update(sample_task)


# ---------------------------------------------------------------------------
# delete
# ---------------------------------------------------------------------------


def test_delete_existing(repo, sample_task):
    repo.add(sample_task)
    repo.delete(sample_task.id)
    with pytest.raises(TaskNotFoundError):
        repo.get(sample_task.id)


def test_delete_missing_raises(repo):
    with pytest.raises(TaskNotFoundError):
        repo.delete("ghost")


def test_count(repo):
    assert repo.count() == 0
    repo.add(Task(title="A"))
    repo.add(Task(title="B"))
    assert repo.count() == 2


# ---------------------------------------------------------------------------
# JSON persistence
# ---------------------------------------------------------------------------


def test_persistence_roundtrip(tmp_path):
    db_file = tmp_path / "tasks.json"
    repo1 = TaskRepository(storage_path=db_file)
    task = repo1.add(Task(title="Persistent task", priority=Priority.HIGH))

    # Load from same file in a new repository instance.
    repo2 = TaskRepository(storage_path=db_file)
    loaded = repo2.get(task.id)
    assert loaded.title == task.title
    assert loaded.priority == task.priority


def test_persistence_file_written(tmp_path):
    db_file = tmp_path / "tasks.json"
    repo = TaskRepository(storage_path=db_file)
    repo.add(Task(title="Write me"))
    assert db_file.exists()
    data = json.loads(db_file.read_text())
    assert len(data) == 1
    assert data[0]["title"] == "Write me"


def test_storage_dir_not_exist_raises(tmp_path):
    with pytest.raises(ValueError, match="does not exist"):
        TaskRepository(storage_path=tmp_path / "ghost_dir" / "tasks.json")
