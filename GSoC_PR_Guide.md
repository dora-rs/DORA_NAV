# Contributing to DORA-RS — PR Guide for GSoC Project #8

## What the Contributors Guide Means for Us

The guide is written for **Rust contributors** to the core `dora-rs` repo. Our GSoC project is different — we're contributing to **`DORA_NAV`**, which is the C++ application layer built *on top* of DORA. So most of the Rust-specific parts (cargo, rustfmt, clippy) don't apply to our code. Here's what **does** apply:

| Rule | Applies to us? | What to do |
|------|---------------|------------|
| Discuss non-trivial changes first | ✅ Yes | Open a GitHub Issue on `dora-rs/DORA_NAV` before writing code |
| Assign yourself with `@dora-bot assign me` | ✅ Yes | Do this on the issue for the local planner feature |
| Don't go inactive for 2 weeks | ✅ Yes | Push commits regularly, comment on your issue |
| `cargo fmt --all` before PR | ❌ Not our code | But **do** run `clang-format` on C++ files |
| CI must pass | ✅ Yes | Make sure your code builds cleanly in Docker |

---

## Step-by-Step: Making a Solid Pull Request

### Step 1 — Fork the repo

```bash
# Fork on GitHub (click Fork button on https://github.com/dora-rs/DORA_NAV)
# Then clone YOUR fork locally:
git clone https://github.com/YOUR_USERNAME/DORA_NAV.git
cd DORA_NAV

# Add the original as upstream so you can sync later:
git remote add upstream https://github.com/dora-rs/DORA_NAV.git
```

---

### Step 2 — Open a GitHub Issue first

Before writing any code, open an issue titled something like:
> **[Feature] Local Obstacle Avoidance Operator using DWA**

In the issue body, describe:
- What algorithm you're implementing (DWA or APF)
- What inputs/outputs the new node will have
- How you'll test it

Then comment `@dora-bot assign me` to claim it.

> This is the most important step — the contributors guide explicitly says *"discuss non-trivial changes first"*. A GSoC feature is definitely non-trivial.

---

### Step 3 — Create a feature branch

```bash
git checkout -b feature/local-obstacle-avoidance
```

Never commit directly to `main`.

---

### Step 4 — Build and test your code inside Docker

After each significant change, verify nothing is broken:

```bash
cd ~/gsoc_dora_project/DORA_NAV
docker compose build   # must complete with no errors
```

Your new node must also not break any existing nodes. Run the full sim to check:
```bash
python3 -m rerun &
docker compose up
```

---

### Step 5 — Format your C++ code

The DORA repo uses `.clang-format` files (you can see one already in `localization/dora-hdl_localization/3rdparty/hdl_ndt_omp/`). Before committing:

```bash
# Install clang-format if needed:
sudo apt install clang-format

# Format all your new files:
find planning/local_planner -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
```

---

### Step 6 — Write a clean commit history

Each commit should do one logical thing. Good examples:
```
feat: add LiDAR pointcloud to obstacle map converter
feat: implement DWA velocity sampling core logic
feat: wrap local planner in DORA node with dataflow YAML
test: add unit tests for collision-free DWA edge cases
docs: add local planner quick-start README
```

Use the `feat:`, `fix:`, `test:`, `docs:` prefixes — this is standard conventional commits format.

```bash
git add planning/local_planner/
git commit -m "feat: add obstacle_map converter from raw LiDAR bytes"
```

---

### Step 7 — Sync with upstream before opening PR

```bash
git fetch upstream
git rebase upstream/main
# Fix any conflicts, then:
git push origin feature/local-obstacle-avoidance
```

---

### Step 8 — Open the Pull Request

On GitHub, open a PR from `YOUR_USERNAME/DORA_NAV:feature/local-obstacle-avoidance` → `dora-rs/DORA_NAV:main`.

**PR title:**
```
feat: Local Obstacle Avoidance operator using DWA (GSoC #8)
```

**PR description template:**
```markdown
## Summary
Implements a local planning operator using the Dynamic Window Approach (DWA)
for real-time obstacle avoidance.

Closes #<issue_number>

## Changes
- `planning/local_planner/` — new DORA node with DWA core logic
- `dataflow_local_avoidance.yml` — wires local planner into the pipeline
- `simulation/mujoco_bridge/models/` — adds obstacle boxes to warehouse scene
- Unit tests for edge cases (narrow corridor, dead-end, free space)

## Demo
[attach screenshot of Rerun viewer showing robot avoiding obstacles]

## Testing
- `docker compose build` completes successfully
- `docker compose up` runs full sim without crashes
- All unit tests pass: `cd planning/local_planner/build && ctest`
```

---

### Step 9 — Respond to review feedback

- Check GitHub notifications daily
- Push fix commits to the same branch — the PR updates automatically
- Don't force-push after review has started unless asked to

---

## What Makes a PR Likely to Be Accepted

| ✅ Do | ❌ Don't |
|------|---------|
| One feature per PR | Bundle multiple unrelated changes |
| Link to the GitHub issue | Open a PR with no prior discussion |
| Include a demo (screenshot/video) | Submit untested code |
| Clean, formatted C++ | Leave debug `printf`s everywhere |
| Describe config parameters in README | Leave undocumented magic numbers |
| Keep Docker build working | Break existing nodes |

---

## Timeline Aligned with PR Strategy

| Phase | What to open |
|-------|-------------|
| Week 1-2 | Issue discussion + design feedback |
| Week 5-6 | Draft PR (mark as `[WIP]`) for early feedback on approach |
| Week 9-10 | Full PR with working sim demo |
| Week 13-14 | Final PR with tests + docs |
