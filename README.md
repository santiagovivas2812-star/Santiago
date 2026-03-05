# Santiago Task Manager

A well-structured Python command-line application for managing tasks, built to
demonstrate best practices in **code quality**, **security**, **performance**, and
**maintainability**.

---

## Features

| Feature | Details |
|---|---|
| Clean architecture | Models → Repository → Service → CLI |
| Type-annotated | Full `mypy --strict` compliance |
| Input validation | Whitelist regex, length limits, domain invariants |
| Safe persistence | Atomic write via `os.replace` prevents data loss |
| Filtered queries | Filter tasks by status and/or priority |
| JSON or table output | `--format json` flag for scripting |
| Full test suite | pytest, 100 % branch coverage target |

---

## Project Layout

```
Santiago/
├── src/
│   └── santiago/
│       ├── __init__.py      # Public API
│       ├── models.py        # Task dataclass + validation
│       ├── exceptions.py    # Custom exception hierarchy
│       ├── repository.py    # In-memory store + JSON persistence
│       ├── service.py       # Business-logic layer
│       └── cli.py           # argparse entry-point
├── tests/
│   ├── test_models.py
│   ├── test_repository.py
│   ├── test_service.py
│   └── test_cli.py
├── pyproject.toml           # Build config, tool settings
├── requirements-dev.txt     # Dev/test dependencies
└── README.md
```

---

## Quick Start

```bash
# Install in editable mode
pip install -e .

# Add a task
santiago add "Write project report" --priority high

# List all tasks
santiago list

# Filter by status
santiago list --status pending

# Mark a task as done
santiago done <task-id>

# Delete a task
santiago delete <task-id>

# JSON output (great for scripts)
santiago --format json list
```

---

## Development

```bash
# Install dev dependencies
pip install -r requirements-dev.txt
pip install -e .

# Run tests
pytest

# Lint + format check
ruff check src tests
ruff format --check src tests

# Type-check
mypy src
```

---

## Design Decisions

### Layered Architecture

```
CLI  →  Service  →  Repository  →  Models
```

* **Models** (`models.py`): pure domain objects with self-validating invariants.
* **Repository** (`repository.py`): thin data-access layer; swappable backend
  (in-memory by default, JSON file when `storage_path` is provided).
* **Service** (`service.py`): orchestrates use-cases; the only place cross-entity
  business rules live.
* **CLI** (`cli.py`): thin presentation layer; delegates everything to the Service.

### Security Considerations

* **Input validation**: titles are matched against a strict whitelist regex; lengths
  are bounded to prevent denial-of-service via huge strings.
* **Atomic file writes**: the JSON store is written to a `.tmp` file and renamed,
  so a crash mid-write never corrupts the database.
* **No `eval` / `exec`**: all deserialisation goes through `json.loads` + explicit
  dataclass construction — no arbitrary code execution.
* **No secrets**: the application stores only user-supplied task data.

### Performance Considerations

* Tasks are held in a `dict` keyed by `id` for O(1) lookup, insert and delete.
* Filtering and sorting are done lazily with generators before materialising the
  final list.
* Persistence is skipped entirely when no `storage_path` is configured.

---

## License

MIT
