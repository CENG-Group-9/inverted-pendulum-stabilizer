---
title: Package Management with uv
impact: MEDIUM
impactDescription: 10-100x faster dependency management
tags: [uv, package-management, pip, virtualenv, lock-file]
---

# Package Management with uv [MEDIUM]

## Description
uv is an extremely fast Python package manager written in Rust by Astral. It's 10-100x faster than pip/pip-tools/virtualenv and replaces them all. Prefer its **project workflow** (`uv add`/`uv lock`/`uv sync`) built around `pyproject.toml` and the native `uv.lock`; a pip-compatible interface (`uv pip ...`) is also available for existing setups.

## Install

```bash
# macOS / Linux
curl -LsSf https://astral.sh/uv/install.sh | sh
# Windows
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

## Project Workflow (recommended)

```bash
# Scaffold a new project (creates pyproject.toml)
uv init

# Add / remove dependencies (updates pyproject.toml AND uv.lock, installs into .venv)
uv add fastapi pydantic
uv add --dev pytest ruff mypy      # dev dependency group
uv remove pydantic

# Resolve and write the lock file explicitly (uv add/sync also keep it current)
uv lock

# Create/sync the environment to match uv.lock exactly
uv sync                 # runtime deps
uv sync --all-extras    # include optional extras
uv sync --locked        # fail if uv.lock is out of date (use in CI)

# Run commands inside the project environment (auto-syncs first)
uv run pytest
uv run python -m myapp

# Upgrade dependencies
uv lock --upgrade                  # all
uv lock --upgrade-package fastapi  # one package
```

## Project Structure

```
project/
├── pyproject.toml   # Source of truth: dependencies, metadata, tool config
├── uv.lock          # Fully-resolved, cross-platform lock file (commit this)
└── .venv/           # Managed environment (do NOT commit)
```

## Key Commands

| Command | Purpose |
|---------|---------|
| `uv init` | Scaffold a project |
| `uv add` / `uv remove` | Add/remove a dependency (edits `pyproject.toml` + `uv.lock`) |
| `uv lock` | Resolve dependencies into `uv.lock` |
| `uv sync` | Make `.venv` match `uv.lock` exactly |
| `uv run` | Run a command in the project environment |
| `uvx` | Run a tool one-off without installing it |
| `uv venv` | Create a bare virtualenv (lower-level) |

## Per-environment usage

| Environment | Command |
|-------------|---------|
| Development | `uv sync` |
| CI | `uv sync --locked` (verifies the lock is current before syncing) |
| Production | `uv sync --locked --no-dev` |

## pip-compatible mode (existing/requirements.txt projects)

If you're not ready to adopt `uv.lock`, uv mirrors the pip-tools workflow:

```bash
uv venv                                          # create virtualenv
uv pip install -e ".[dev]"                       # install
uv pip compile pyproject.toml -o requirements.lock --generate-hashes
uv pip sync requirements.lock                    # exact install
```

For new projects prefer the project workflow above; `uv.lock` is cross-platform and is
maintained automatically by `uv add`/`uv sync`, so you don't recompile by hand.

## Notes
- uv uses a global cache, making repeated installs instant
- Commit `uv.lock` (project workflow) or your `requirements.lock` (pip mode); never commit `.venv/`
- `uv sync --locked` in CI fails if `uv.lock` is stale. Use `uv sync --frozen` only when you intentionally want to skip freshness checks and trust the existing lock file.
- `uvx` is great for one-off tool execution (e.g. `uvx ruff check .`)
- Run `uv lock --upgrade` regularly for security patches

## References
- [uv Documentation](https://docs.astral.sh/uv/)
- [uv GitHub](https://github.com/astral-sh/uv)
