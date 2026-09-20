## What changed

<!-- One or two sentences. What does this do that main does not? -->

## Why

<!-- The problem, scenario, or issue this addresses. Link it: Closes #12 -->

## How I tested it

<!-- Commands you actually ran, and what you saw. -->

```bash
python -m pytest -q
python main.py --scenario scenarios/normal.yaml --mode adaptive
```

## Checklist

- [ ] `python -m pytest -q` passes locally
- [ ] New behaviour has a test
- [ ] `--mode baseline` still runs (results stay comparable)
- [ ] No generated output, `.venv/`, secrets or backups in the diff
- [ ] Docs updated if a CLI flag or config key changed

## Screenshots / dashboard

<!-- If this changes the dashboard or Mission Planner view, attach an image. -->
