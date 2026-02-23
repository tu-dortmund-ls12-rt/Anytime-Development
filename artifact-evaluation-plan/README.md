# Artifact Evaluation Plan

Revised plan for the RTAS 2026 artifact evaluation of "Anytime ROS 2: Timely Task Completion in Non-Preemptive Robotic Systems."

**Deadline:** February 26, 2026

**AE Criteria (1-5 scale each, need 3+ average with no 1s):**

1. **Coverage for Repeatability** - Can results be reproduced?
2. **Instructions** - Are setup/reproduction steps clear?
3. **Quality** - Is code documented, tested, well-structured?

## Schedule (3 days)

### Day 1 (Tue Feb 24) - Correctness + Paths

- [x] Step 1: Fix hardcoded paths everywhere (~3h) **[CRITICAL]**
- [ ] Step 2: GPU sync fix (~3h) **[CRITICAL]**
- [ ] Step 3: C++ correctness bugs (~1.5h)

### Day 2 (Wed Feb 25) - Tests + Automation

- [ ] Step 4: Unit tests (~3h)
- [ ] Step 5: Smoke test + simplified experiments (~1.5h)
- [ ] Step 6: Bash wrapper scripts for automation (~1.5h)
- [ ] Step 7: docker-compose.yml (~0.5h)

### Day 3 (Thu Feb 26) - Documentation + Polish

- [ ] Step 8: Code quality fixes + metadata (~1h)
- [ ] Step 9: Documentation (~3h)
- [ ] Step 10: Artifact Evaluation guide (~1.5h)
- [ ] Step 11: Final polish (~1h)

## What Was Cut (from old 12-step plan) and Why

| Old Step | Cut? | Reason |
| -------- | ---- | ------ |
| 07: Python CLI (anytime-eval) | **CUT** | Replaced by simple bash wrapper scripts |
| 08: CI/CD Pipeline | **CUT** | Doesn't affect AE scoring |
| 06: Docker Restructuring | **SIMPLIFIED** | Keep existing Dockerfiles, just add docker-compose.yml |
| 04: GPU Integration Tests | **CUT** | Requires GPU hardware evaluators may not have |
| 12.1: Pre-built GHCR images | **CUT** | Risk of registry issues |
| 12.5: Performance baselines | **CUT** | Not needed for AE |
| 12.6: Badges | **CUT** | Premature, add after AE passes |

## Dependency Graph

```text
Day 1: Step 1 (paths) -------------------------+
       Step 2 (GPU sync) ----------------------+
       Step 3 (C++ bugs) ----------------------+
                                                |
Day 2: Step 4 (unit tests) <- depends on 2,3 --+
       Step 5 (smoke+quick) <- depends on 1,4 -+
       Step 6 (wrapper scripts) <- depends on 1 +
       Step 7 (docker-compose) -- independent --+
                                                |
Day 3: Step 8 (quality+metadata) -- independent +
       Step 9 (documentation) <- depends on 2 --+
       Step 10 (AE guide) <- depends on 6,9 ----+
       Step 11 (polish) <- depends on all -------+
```

## AE Scoring Targets

| Criterion | Target | How |
| --------- | ------ | --- |
| 1: Repeatability | 4 | All figures reproducible via scripts. Quick + full modes. Path fixes ensure portability. |
| 2: Instructions | 4-5 | Single-command reproduction per figure. Architecture + components + "how to modify" docs. |
| 3: Quality | 4 | All files documented. Unit tests for core + Monte Carlo. System-level smoke test. |

## Verification Checklist (end of Day 3)

- [ ] `grep -rn "/home/vscode" experiments/ packages/` returns 0 results outside Dockerfiles
- [ ] `colcon build --symlink-install` succeeds from clean state
- [ ] `colcon test --packages-select anytime_core anytime_monte_carlo` - all pass
- [ ] `./scripts/smoke_test.sh` passes in <2 min
- [ ] `./scripts/run_all.sh --quick --cpu-only` completes successfully
- [ ] `./scripts/reproduce_figure.sh 5a` generates expected plot
- [ ] `docker compose build anytime-cpu` succeeds
- [ ] All markdown docs render correctly
- [ ] `main.pdf` present at repo root
- [ ] `LICENSE` file present
- [ ] `ARTIFACT_EVALUATION.md` has complete figure-to-command mapping
