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

### Fuzzing

CI fuzzes nightly, not on pull requests. If you change `proto/framing/`, run the fuzzer before
pushing (needs `clang` and `libclang-rt-dev`):

```bash
CC=clang CXX=clang++ colcon build --packages-up-to lagrange_proto --cmake-args \
  -DBUILD_TESTING=OFF -DLAGRANGE_BUILD_FUZZERS=ON "-DLAGRANGE_SANITIZER=address;undefined"
install/lagrange_proto/lib/lagrange_proto/fuzz_frame_reader -max_total_time=60 -max_len=8192
```

If Clang fails to link with `cannot find -lstdc++`, it has picked a GCC version whose C++
library is not installed. Add `--gcc-install-dir=/usr/lib/gcc/x86_64-linux-gnu/15` to `CFLAGS`,
`CXXFLAGS` and `LDFLAGS`.

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

## Changelog and releases

`CHANGELOG.md` is generated. Do not edit it in a pull request: the title of your pull request
(a Conventional Commit) becomes its changelog line when it is squash-merged into `dev`.
`feat`, `fix`, `perf`, `revert`, `docs` and `build` appear in the changelog; other types are
hidden. A `!` after the type marks a breaking change.

The `release-please` workflow keeps one release pull request open against `dev` with the next
version, the new changelog section and the version in every `package.xml`. Merging it tags the
release; `dev` is then merged into `main`.
