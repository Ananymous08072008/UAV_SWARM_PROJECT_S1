# Contributing

How to work on this repository. Please read this before your first change.

## One-time setup

```bash
git clone https://github.com/Ananymous08072008/UAV_SWARM_PROJECT_S1.git
cd UAV_SWARM_PROJECT_S1

python -m venv .venv
.venv\Scripts\activate          # Windows;  source .venv/bin/activate on Linux/macOS
pip install -r requirements.txt

python -m pytest -q              # confirm all tests pass before you change anything
```

Set your identity once, so commits are attributed to you:

```bash
git config --global user.name "Your Name"
git config --global user.email "your@email.com"
```

## The rule: never commit directly to `main`

`main` must always be runnable and green. All work happens on a branch and
arrives through a pull request.

## Day-to-day workflow

```bash
# 1. Start from an up-to-date main
git switch main
git pull

# 2. Create a branch for your task
git switch -c feat/relay-handover

# 3. Work, then see what you changed
git status
git diff

# 4. Stage and commit in small logical pieces
git add swarm/relay_selector.py tests/test_relay_selection.py
git commit -m "swarm: pick relays by ETX instead of raw distance"

# 5. Run the tests before pushing
python -m pytest -q

# 6. Push your branch
git push -u origin feat/relay-handover
```

Then open a pull request on GitHub, describe what changed and how you tested
it, and request a review. Once it is approved and CI is green, merge it.

After merging, clean up:

```bash
git switch main
git pull
git branch -d feat/relay-handover
```

## Branch naming

| Prefix     | Use for                          | Example                          |
|------------|----------------------------------|----------------------------------|
| `feat/`    | a new capability                 | `feat/multi-hop-ferrying`        |
| `fix/`     | a bug fix                        | `fix/battery-drain-at-speed-4`   |
| `docs/`    | documentation only               | `docs/mission-planner-udp-setup` |
| `test/`    | tests only                       | `test/scenario-coverage`         |
| `refactor/`| restructuring, no behaviour change | `refactor/split-world-step`    |

## Commit messages

One line, present tense, says what changed and why. Prefix with the area.

```
swarm: recall UAVs when deadline slips past remaining flight time
telemetry: hold position updates at 5 Hz regardless of sim speed
fix: stop relay re-planning from thrashing on hysteresis boundary
```

Avoid `update`, `fixes`, `changes`, `asdf`. Six months from now the log has to
still make sense.

## Before you open a pull request

- [ ] `python -m pytest -q` passes locally
- [ ] New behaviour has a test
- [ ] No generated output, `.venv/`, secrets, or `.zip` backups in the diff (`git status` is clean)
- [ ] Both `--mode adaptive` and `--mode baseline` still run
- [ ] Docs updated if you changed a CLI flag or a config key

## Project conventions

- Config lives in `config/parameters.yaml`. Do not hardcode tunable numbers.
- Keep the two networks separate: the simulated **swarm research network** and
  the **MAVLink telemetry link** used only for Mission Planner display.
- Anything that disables a research contribution must be gated behind
  `--mode baseline` so results stay comparable.
- Every state change should publish an event, so the dashboard and logs stay in sync.

## Resolving a conflict

If `git pull` reports a conflict, open the marked files, keep the correct code,
delete the `<<<<<<<`, `=======` and `>>>>>>>` markers, then:

```bash
git add <the-file>
git commit
```

Ask before force-pushing anything. `git push --force` on a shared branch
destroys teammates' work.
