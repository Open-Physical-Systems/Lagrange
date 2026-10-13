<!-- Title must be a Conventional Commit: type(scope): subject   e.g. feat(viewer): add scope cursors -->
<!-- Target branch: dev. Only release PRs (from dev, release/* or hotfix/*) target main. -->

## What and why

## How it was tested
<!-- Commands, new tests, screenshots for viewer changes. -->

## Checklist
- [ ] Branch is named `<type>/<kebab-description>` and targets `dev`.
- [ ] Commits are signed off (DCO, `git commit -s`).
- [ ] `pre-commit run --all-files` passes (once the hooks exist).
- [ ] Tests added or updated; sanitizer builds considered.
- [ ] Docs updated (user docs, design docs, CLAUDE.md if an invariant, command or phase changed).
- [ ] New dependency? License checked and recorded in `docs/design/dependencies.md`.
- [ ] New or changed physical model? Validation test and reference included.
- [ ] Feature has defined behaviour on Tier 0, 1 and 2.
- [ ] No file larger than 1 MB.

## Stable surfaces
- [ ] This PR does **not** change `schemas/`, `core/plugin_abi/`, `proto/` or `profiles/schema.json`.
- [ ] It does, and: RFC link or justification here → ______ ; migration added in `tools/` ; label
      `schema-break` / `proto-break` / `abi-break` applied if breaking (needs 2 approvals).

## Invariants (CLAUDE.md §3)
- [ ] No module includes or calls another module; communication is through the signal bus.
- [ ] No USD access inside `step()`; no `float` world positions.
- [ ] Viewer changes do not link simulation modules and contain no web technology.
