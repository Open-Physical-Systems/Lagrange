# Contributing to Lagrange

Read [CLAUDE.md](CLAUDE.md) first. Despite its name it is the project's rule book: architecture
invariants, format choices, code conventions and the Git workflow.

## Workflow

1. Branch from `dev`. `main` is for releases only.
2. Name the branch `<type>/<kebab-description>`, for example `feat/viewer-scope-panel`.
   Types: `feat`, `fix`, `docs`, `ci`, `chore`, `refactor`, `perf`, `test`, `build`, `revert`.
3. Write Conventional Commits and sign every commit off (`git commit -s`). The sign-off certifies
   the [Developer Certificate of Origin](https://developercertificate.org/).
4. Open a pull request against `dev`. The PR title must be a Conventional Commit, because pull
   requests are squash-merged and the title becomes the commit message.
5. CI must pass. Changes to `schemas/`, `core/plugin_abi/` or `proto/` need an RFC or an explicit
   note in the PR, and two approvals.

Commit scopes follow the directory tree: `core`, `gpu`, `mod/<domain>`, `viewer`, `proto`,
`bridge/<name>`, `io`, `schemas`, `profiles`, `server`, `mock`, `python`, `docs`, `ci`,
`packaging`, `deps`.

## Before you push

```bash
pre-commit run --all-files
colcon build && colcon test && colcon test-result --verbose
```

`pre-commit` is in Ubuntu 26.04 (`sudo apt install pre-commit`); run `pre-commit install` once
in your clone.

## Rules that reviewers will check

- Modules talk only through the signal bus. No module includes another module.
- Every physical quantity in an interface has a unit. SI by default.
- A new physical model comes with a test and a reference it is checked against.
- A new dependency has its license recorded in `docs/design/dependencies.md`.
- No file larger than 1 MB.
- No web technology anywhere in the product.

## Design changes

Larger changes start as an RFC in `rfcs/`, numbered in sequence, with motivation, design,
alternatives considered and unresolved questions.
