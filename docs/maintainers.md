# Maintainers Guide

This document is for TurtleRabbits members who help maintain TeamControl. It is a practical checklist for reviewing changes, keeping setup reliable, and preparing releases.

## Maintainer Responsibilities

- Keep `README.md`, `CONTRIBUTING.md`, and the getting started docs accurate.
- Review pull requests for correctness, readability, and safety.
- Check that setup still works for new members.
- Keep simulator, SSL-Vision, Game Controller, and robot network configuration documented.
- Build and test release binaries before sharing them.
- Avoid committing machine-specific config unless it is a safe default.

## Before Merging a PR

Check:

- Does the code run locally?
- Does `pytest` pass?
- Are docs updated if behavior, setup, or config changed?
- Does the change affect grSim, SSL-Vision, Game Controller, or robot networking?
- Are config changes safe for other machines?
- Are generated files excluded unless intentionally committed?

## Testing Checklist

Run the test suite:

```shell
pytest
```

For UI changes:

```shell
python ui_main.py
```

For headless mode:

```shell
python main.py --mode <some mode>
```

For grSim changes:

- Start grSim.
- Confirm the command port.
- Confirm the vision port.
- Check `src/TeamControl/utils/ipconfig.yaml`.
- Verify robots receive commands in simulation.

## Release Checklist

Build the Windows binary:

```powershell
.\scripts\build_windows.ps1
```

Test the generated apps:

```powershell
.\dist\TeamControl\TeamControl.exe
.\dist\TeamControlCLI\TeamControlCLI.exe --mode goalie
```

Before sharing a release:

- Zip the whole `dist\TeamControl` folder for UI users.
- Test on a clean Windows machine if possible.
- Confirm the app starts without a Python environment.
- Confirm config files are present or editable as expected.
- Note known limitations in the release notes.

## Versioning

Use simple versions until the project needs a stricter release process:

```text
0.1.0
0.2.0
0.3.0
```

Suggested meaning:

- Patch: docs, small fixes, no behavior change.
- Minor: new features, UI changes, strategy changes.
- Major: breaking config, architecture, or API changes.

## Branches

Recommended flow:

- `main`: stable enough for new members.
- feature branches: work in progress.
- pull requests: reviewed before merge.

## Files To Watch

- `README.md`
- `CONTRIBUTING.md`
- `docs/gettting_started.md`
- `pyproject.toml`
- `scripts/build_windows.ps1`
- `src/TeamControl/utils/ipconfig.yaml`
- `main.py`
- `ui_main.py`

## Generated Files

Do not commit generated build output unless there is a specific reason.

Usually exclude:

```text
build/
dist/
*.spec
.venv/
__pycache__/
```

Release binaries should go in GitHub Releases, not normal source commits.

## Network Safety

Be careful when changing default IPs, ports, team colors, or grSim settings. These settings affect whether the software talks to simulation or real robots.

Document any change to:

- grSim IP/port
- SSL-Vision multicast group/port
- Game Controller multicast group/port
- robot IP mapping
- `send_to_grSim`
- `use_grSim_vision`

## Emergency Rollback

If a change breaks setup for new members:

1. Revert or fix the change quickly.
2. Update the issue or PR with what broke.
3. Add a test or doc note if it prevents the same mistake.
