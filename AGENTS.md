# Repository Guidance

## Commands

- Prefix shell commands with `rtk`. See `/Users/a0000/.codex/RTK.md`.

## Cross-Platform Development

This repository keeps the platform-neutral product and the macOS support layer
separate. Do not add macOS packaging or dependency support back to `main`
without an explicit decision to change this policy.

### Branch Roles

- `main` contains platform-neutral product code and is the target for generic
  features and fixes.
- `develop-MacOS` contains `main` plus the macOS Apple Silicon integration
  layer: platform-specific build configuration, dependency compatibility,
  packaging, and macOS-only tests.
- `feature/<name>` branches from `main` and targets `main`.
- `macos/<name>` branches from `develop-MacOS` and targets `develop-MacOS`.
- `sync/main-<short-sha>` is a temporary integration branch used only to bring
  a specific `main` revision into `develop-MacOS`.

### Daily Rules

- Keep GUI, algorithms, and product behavior platform-neutral whenever
  possible. Prefer a small macOS overlay over a long-lived source fork.
- Do not open a pull request from `develop-MacOS` to `main`.
- For generic work that needs macOS or camera validation, validate it on a
  temporary branch based on `develop-MacOS`, then create a clean `main` PR
  containing only the generic commits.
- Keep macOS-only changes in `macos/*` branches and review them against
  `develop-MacOS`.
- Never force-push, rebase, or directly merge into the shared
  `develop-MacOS` branch. Rebase is allowed only on an unshared local feature
  branch before its pull request is opened.

### Synchronizing `main` Into `develop-MacOS`

Use a reviewable synchronization pull request whenever `main` updates need to
reach the macOS integration line:

1. Fetch both branches and create `sync/main-<short-sha>` from the current
   `develop-MacOS`.
2. Integrate the selected `main` revision into that temporary branch.
3. Resolve conflicts by keeping generic behavior owned by `main` and macOS
   build/package glue owned by `develop-MacOS`.
4. Run the relevant macOS build and test checks.
5. Open a pull request from `sync/main-<short-sha>` to `develop-MacOS`.

Do not automate a rebase of the shared branch. Future automation may create
the synchronization branch and pull request, but it must leave conflict
resolution, test review, and merge approval visible to maintainers.

### Current Transition

`main` historically reverted the original macOS commits. Before the recurring
synchronization process begins, establish `develop-MacOS` as an explicit
overlay on the current `main` tip by replaying the macOS commits and the
macOS-specific compatibility changes in a dedicated, reviewed bootstrap PR.
