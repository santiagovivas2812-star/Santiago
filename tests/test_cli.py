"""Tests for the CLI entry-point."""

from __future__ import annotations

import json

import pytest

from santiago.cli import main


@pytest.fixture()
def db(tmp_path):
    return str(tmp_path / "tasks.json")


def run(*args, db):
    return main(["--db", db, *args])


# ---------------------------------------------------------------------------
# add
# ---------------------------------------------------------------------------


def test_add_task(db, capsys):
    rc = run("add", "Buy milk", db=db)
    assert rc == 0
    out = capsys.readouterr().out
    assert "Task created" in out


def test_add_task_json_output(db, capsys):
    rc = run("--format", "json", "add", "Fix bug", db=db)
    assert rc == 0
    data = json.loads(capsys.readouterr().out)
    assert data["title"] == "Fix bug"
    assert data["status"] == "pending"


def test_add_task_invalid_title(db, capsys):
    rc = run("add", "", db=db)
    assert rc == 1
    assert "error" in capsys.readouterr().err.lower()


# ---------------------------------------------------------------------------
# list
# ---------------------------------------------------------------------------


def test_list_empty(db, capsys):
    rc = run("list", db=db)
    assert rc == 0
    assert "No tasks found" in capsys.readouterr().out


def test_list_shows_tasks(db, capsys):
    run("add", "Task one", db=db)
    run("add", "Task two", db=db)
    capsys.readouterr()
    rc = run("list", db=db)
    assert rc == 0
    out = capsys.readouterr().out
    assert "Task one" in out
    assert "Task two" in out


def test_list_filter_status(db, capsys):
    run("add", "Pending task", db=db)
    capsys.readouterr()
    rc = run("list", "--status", "done", db=db)
    assert rc == 0
    assert "No tasks found" in capsys.readouterr().out


# ---------------------------------------------------------------------------
# done
# ---------------------------------------------------------------------------


def test_done_command(db, capsys):
    run("--format", "json", "add", "Complete me", db=db)
    task_id = json.loads(capsys.readouterr().out)["id"]
    rc = run("done", task_id, db=db)
    assert rc == 0
    assert "marked as done" in capsys.readouterr().out


def test_done_not_found(db, capsys):
    rc = run("done", "nonexistent-id", db=db)
    assert rc == 1


# ---------------------------------------------------------------------------
# delete
# ---------------------------------------------------------------------------


def test_delete_command(db, capsys):
    run("--format", "json", "add", "Delete me", db=db)
    task_id = json.loads(capsys.readouterr().out)["id"]
    rc = run("delete", task_id, db=db)
    assert rc == 0


def test_delete_not_found(db, capsys):
    rc = run("delete", "ghost-id", db=db)
    assert rc == 1
