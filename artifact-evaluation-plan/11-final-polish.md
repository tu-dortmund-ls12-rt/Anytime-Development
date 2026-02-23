# Step 11: Final Polish

**Status:** Pending
**Day:** 3 (Thu Feb 26)
**Priority:** MEDIUM
**Time:** ~1h
**Dependencies:** All previous steps

## Tasks

### LICENSE file

- New file: `LICENSE` - Apache 2.0 full text

### requirements.txt

- New file: `requirements.txt` - pin pandas, numpy, matplotlib, pyyaml, babeltrace2

### Update .gitignore

- Add: `__pycache__/`, `*.pyc`, `*.egg-info/`, `.pytest_cache/`, `dist/`

### Update README.md

- Add link to `ARTIFACT_EVALUATION.md`
- Add link to `docs/` directory
- Add citation block
- Add license section
- Update Quick Start to use docker-compose

### Script permissions

```bash
find . -name "*.sh" -exec chmod +x {} \;
```

### Code formatting

```bash
# Run clang-format on modified C++ files
```

### Final verification

```bash
# Clean build
cd packages && rm -rf build/ install/ log/
colcon build --symlink-install
colcon test --packages-select anytime_core anytime_monte_carlo
colcon test-result --verbose
./scripts/smoke_test.sh
```
