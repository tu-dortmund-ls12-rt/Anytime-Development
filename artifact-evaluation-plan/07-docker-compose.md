# Step 7: docker-compose.yml

**Status:** Completed
**Day:** 2 (Wed Feb 25)
**Priority:** MEDIUM
**Time:** ~0.5h
**Dependencies:** None

## Why

Simplifies container management for evaluators.

**AE Impact:** Criterion 2 (Instructions) - easier setup.

## New Files

### `docker-compose.yml`

```yaml
services:
  anytime-gpu:
    build:
      context: .
      dockerfile: .devcontainer/linux/Dockerfile
    runtime: nvidia
    volumes:
      - .:/home/vscode/workspace
    environment:
      - NVIDIA_VISIBLE_DEVICES=all

  anytime-cpu:
    build:
      context: .
      dockerfile: .devcontainer/linux-no-hardware/Dockerfile
    volumes:
      - .:/home/vscode/workspace
```

### `.dockerignore`

Exclude build artifacts, traces, results, .git, etc.

## Verification

```bash
docker compose build anytime-cpu
docker compose run --rm anytime-cpu bash -c "cd packages && colcon build"
```
