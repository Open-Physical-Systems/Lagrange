# Repository, CI, release and security plan

Status: plan for review. This file contains the full draft of every configuration and workflow
file. Nothing here has been created in the repository yet.

Pinning rule: tool versions seen during verification are named in comments. Pre-commit `rev:`
values and GitHub Action commit SHAs were **not** resolved in the planning task and are written
as `TODO(verify)`. They must be resolved from the upstream repositories when the files are
created (roadmap issue 0.1-A2 and A4). A file with an unresolved `TODO(verify)` pin must not be
merged.

## 1. Git workflow

| Rule | Setting |
|---|---|
| Model | `main` is release-only. `dev` is the integration branch; all work branches start from `dev` and merge into `dev`. A release is a PR from `dev` to `main` |
| Branch names | Conventional: `<type>/<kebab-description>`, type in `feat`, `fix`, `docs`, `ci`, `chore`, `refactor`, `perf`, `test`, `build`, `revert`; plus `release/x.y` and `hotfix/…`. Checked by `.github/workflows/pr-checks.yml` |
| Merging | Squash merge into `dev`. Merge commit from `dev` into `main` at release time. Merge queue on `dev` when the plan allows it |
| Commits | Conventional Commits. The squash commit title is the PR title, so the PR title is what is checked |
| Scopes | `core`, `gpu`, `mod/<domain>`, `viewer`, `proto`, `bridge/<name>`, `io`, `schemas`, `profiles`, `server`, `mock`, `python`, `docs`, `ci`, `packaging`, `deps` |
| Sign-off | DCO on every commit (`git commit -s`) |
| Tags | `vX.Y.Z`, signed (Sigstore gitsign or GPG; decide with the release workflow) |
| Release branches | From v1.0: `release/1.x`, supported for the life of ROS 2 Lyrical (May 2031). Fixes land on `main` first and are cherry-picked by a bot label `backport/1.x` |
| Large files | Nothing over 1 MB. No Git LFS in the main repo (Free plan includes 10 GiB storage and 10 GiB bandwidth per month; a public repo cloned by CI would use that quickly). Large assets live in release bundles of a separate `lagrange-assets` repo and are fetched by checksum |

Decision of 2026-10-13: this `dev`/`main` model replaces the trunk-based model first drafted
here. Where later sections say "`main`" for day-to-day CI triggers, protection and
`buf breaking` baselines, read `dev`; `main` keeps the same protection plus "PRs only from
`dev`, `release/*` or `hotfix/*`".

### Branch protection for `main` and `dev` (repository ruleset)

| Setting | Value |
|---|---|
| Require a pull request | yes |
| Required approvals | 1; 2 for paths owned by `@Open-Physical-Systems/core-maintainers` in CODEOWNERS (`schemas/`, `core/plugin_abi/`, `proto/`) through "require review from code owners" plus a second-approval check job |
| Dismiss stale approvals on new commits | yes |
| Require conversation resolution | yes |
| Require status checks | `lint`, `commits`, `build-test (amd64)`, `build-test (arm64)`, `sanitizers (asan-ubsan)`, `sanitizers (tsan)`, `viewer-headless`, `proto`, `docs`, `reuse` |
| Require merge queue | yes; merge method squash; queue also runs `gpu-tests` and `determinism` |
| Require linear history | yes |
| Require signed commits | no for contributors (DCO is the requirement); tags are signed |
| Force pushes, deletions | blocked |
| Bypass | nobody, including administrators |

Merge queue availability depends on the repository owner type and plan (TODO(verify), open
question Q19). If it is unavailable, the fallback is "require branches to be up to date" plus the
GPU and determinism jobs on `push` to `main` with automatic revert on failure.

## 2. Files to create

```
.editorconfig  .gitattributes  .gitignore
.clang-format  .clang-tidy  .pre-commit-config.yaml  .yamllint  .typos.toml
pyproject.toml                # ruff, mypy, pytest, commitizen
buf.yaml  buf.gen.yaml
CMakePresets.json  colcon.defaults.yaml
LICENSE  LICENSES/Apache-2.0.txt (+ one file per third-party license used)  REUSE.toml
README.md  CONTRIBUTING.md  CODE_OF_CONDUCT.md  SECURITY.md  GOVERNANCE.md  CHANGELOG.md
release-please-config.json  .release-please-manifest.json
packaging/rosdep/lagrange.yaml
.github/
├── CODEOWNERS
├── ISSUE_TEMPLATE/{bug.yml, feature.yml, robot-class.yml, config.yml}
├── PULL_REQUEST_TEMPLATE.md
├── labels.yml
├── renovate.json
├── dependabot.yml            # not used; Renovate covers it (file absent on purpose)
└── workflows/{ci.yml, merge-queue.yml, nightly.yml, release.yml, codeql.yml,
               scorecard.yml, labels.yml, docs.yml}
```

`commitlint.config.*` is listed in the task brief. commitlint needs Node, which the project
otherwise avoids even in developer tooling (CLAUDE.md §12.3). The plan uses **commitizen**
(Python, MIT, 4.19.2 on PyPI) instead. Both configurations are given in section 6 so that either
can be chosen (open question Q13).

## 3. Editor, Git and ignore files

### `.editorconfig`

```ini
root = true

[*]
charset = utf-8
end_of_line = lf
insert_final_newline = true
trim_trailing_whitespace = true
indent_style = space
indent_size = 4
max_line_length = 100

[*.{yml,yaml,json,usda,proto,toml,md}]
indent_size = 2

[*.md]
trim_trailing_whitespace = false

[{CMakeLists.txt,*.cmake}]
indent_size = 4

[Makefile]
indent_style = tab

[*.{py,pyi}]
max_line_length = 100
```

### `.gitattributes`

```gitattributes
* text=auto eol=lf

*.cpp   text diff=cpp
*.hpp   text diff=cpp
*.h     text diff=cpp
*.c     text diff=cpp
*.slang text diff=cpp
*.py    text diff=python
*.md    text diff=markdown
*.usda  text
*.proto text
*.yaml  text
*.yml   text
*.json  text

*.usdc  binary
*.usdz  binary
*.mcap  binary
*.vdb   binary
*.nvdb  binary
*.fmu   binary
*.png   binary
*.jpg   binary
*.ktx2  binary
*.ttf   binary
*.otf   binary
*.spv   binary

schemas/generated/** linguist-generated=true
proto/gen/**         linguist-generated=true
tests/**/golden/**   linguist-generated=true -diff
CHANGELOG.md         merge=union
```

### `.gitignore`

```gitignore
# colcon
/build*/
/install*/
/log/
# CMake presets output
/out/
CMakeUserPresets.json
compile_commands.json
# Python
__pycache__/
*.py[cod]
.mypy_cache/
.ruff_cache/
.pytest_cache/
*.egg-info/
.venv/
# Editors
.vscode/
.idea/
*.swp
# Generated
*.spv
/proto/gen/
/docs/_build/
# Runs and recordings
/runs/
*.mcap
!tests/**/fixtures/*.mcap
!assets/**/*.mcap
# Fetched assets and vendor caches
/.cache/
/assets/_fetched/
# Viewer user state
imgui.ini
```

## 4. C++ tooling

### `.clang-format`

The reference in CLAUDE.md §12.1 with three additions: categories for quoted includes, for ROS
and for the viewer's third-party headers; the rest is unchanged.

```yaml
# clang-format 21 (Ubuntu 26.04: 21.1.6). Pin the major version in pre-commit and CI.
BasedOnStyle: Google
Language: Cpp
Standard: c++20            # matches the project's language level (see open question Q1)
ColumnLimit: 100
IndentWidth: 4
AccessModifierOffset: -4
PointerAlignment: Left
DerivePointerAlignment: false
IncludeBlocks: Regroup
IncludeCategories:          # first matching regex wins, so order matters
  - Regex: '^"'              # this target's own headers, first block after the main include
    Priority: 0
  - Regex: '^<lagrange/'      # project headers last
    Priority: 4
  - Regex: '^<(rclcpp|rclcpp_components|rcl|rmw|rosidl|hardware_interface|controller_manager|pluginlib|tf2|tf2_ros|ament_index_cpp|[a-z0-9_]+_msgs)/'   # ROS
    Priority: 3
  - Regex: '^<(pxr|PxPhysics|mujoco|Jolt|openvdb|nanovdb|embree4|vulkan|vk_mem_alloc|volk|SDL3|imgui|implot|google/protobuf|mcap|yaml-cpp|nlohmann|fmt|spdlog|Eigen|gtest|gmock|benchmark)[/.]'   # third-party
    Priority: 2
  - Regex: '^<.*>'           # std
    Priority: 1
AllowShortFunctionsOnASingleLine: Inline
BreakConstructorInitializers: BeforeComma
SortIncludes: CaseSensitive
```

Slang files are formatted with the same file through a pre-commit hook restricted to `\.slang$`.
Whether clang-format 21 treats Slang syntax (attributes, `interface`, generics) acceptably must
be checked on the first real kernels; if not, Slang files are excluded from formatting and only
compile-checked (CLAUDE.md §12.4 already flags this).

### `.clang-tidy`

One correction to CLAUDE.md §12.2: `HeaderFilterRegex` is matched against the header's path as
the compiler sees it, which is normally absolute. A pattern anchored with `^core/` would match no
header, and header diagnostics would be silently skipped [High, from clang-tidy's documented
behaviour]. The pattern below matches the directory anywhere in the path.

```yaml
Checks: >
  -*,
  bugprone-*,
  cppcoreguidelines-*,
  modernize-*,
  performance-*,
  readability-*,
  concurrency-*,
  -modernize-use-trailing-return-type,
  -readability-magic-numbers,
  -cppcoreguidelines-avoid-magic-numbers,
  -readability-identifier-length,
  -bugprone-easily-swappable-parameters
WarningsAsErrors: 'bugprone-*,concurrency-*,performance-*'
HeaderFilterRegex: '.*/(core|gpu|modules|bridges|io|viewer|server|mock_server)/.*'
ExcludeHeaderFilterRegex: '.*/(vendor|build[^/]*|install[^/]*|proto/gen|schemas/generated)/.*'
FormatStyle: file
CheckOptions:
  - { key: readability-identifier-naming.ClassCase,        value: CamelCase }
  - { key: readability-identifier-naming.StructCase,       value: CamelCase }
  - { key: readability-identifier-naming.FunctionCase,     value: camelBack }
  - { key: readability-identifier-naming.VariableCase,     value: camelBack }
  - { key: readability-identifier-naming.PrivateMemberSuffix, value: '_' }
  - { key: readability-identifier-naming.NamespaceCase,    value: lower_case }
  - { key: readability-identifier-naming.EnumConstantCase, value: CamelCase }
  - { key: readability-identifier-naming.ConstexprVariableCase, value: camelBack }
  - { key: readability-function-cognitive-complexity.Threshold, value: '40' }
```

The two extra disabled checks (`identifier-length`, `easily-swappable-parameters`) are proposals:
physics code uses names like `R`, `L`, `dt` and functions with several `double` parameters by
nature. `ExcludeHeaderFilterRegex` exists in recent clang-tidy versions; TODO(verify) that
clang-tidy 21 accepts the key. The naming rules follow the reference code in CLAUDE.md §11
(`preferredStep`, `joints_`); the constants in `lagrange::units` (`N_m`, `rad_per_s`) are exempted
with `// NOLINT` at the namespace or by a naming-rule exception to be settled in issue 0.1-A2.

### `CMakePresets.json`

Presets are for IDEs and single-package work. The normal build is `colcon build`, which reads
`colcon.defaults.yaml`. Both set the same options through one CMake module so they cannot drift.

```json
{
  "version": 6,
  "cmakeMinimumRequired": { "major": 3, "minor": 28, "patch": 0 },
  "configurePresets": [
    {
      "name": "base",
      "hidden": true,
      "generator": "Ninja",
      "binaryDir": "${sourceDir}/out/${presetName}",
      "cacheVariables": {
        "CMAKE_EXPORT_COMPILE_COMMANDS": "ON",
        "CMAKE_CXX_STANDARD": "20",
        "CMAKE_CXX_STANDARD_REQUIRED": "ON",
        "CMAKE_CXX_EXTENSIONS": "OFF",
        "LAGRANGE_WARNINGS_AS_ERRORS": "ON"
      }
    },
    { "name": "dev", "inherits": "base", "displayName": "Developer (RelWithDebInfo)",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "RelWithDebInfo", "CMAKE_CXX_COMPILER_LAUNCHER": "ccache" } },
    { "name": "release", "inherits": "base", "displayName": "Release",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "Release", "CMAKE_INTERPROCEDURAL_OPTIMIZATION": "ON" } },
    { "name": "deterministic", "inherits": "release", "displayName": "Release, deterministic FP flags",
      "cacheVariables": { "LAGRANGE_DETERMINISTIC": "ON" } },
    { "name": "asan", "inherits": "base", "displayName": "ASan + UBSan",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "Debug", "LAGRANGE_SANITIZER": "address;undefined" } },
    { "name": "tsan", "inherits": "base", "displayName": "TSan",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "Debug", "LAGRANGE_SANITIZER": "thread" } },
    { "name": "ci-amd64", "inherits": "base", "displayName": "CI amd64",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "RelWithDebInfo", "CMAKE_CXX_COMPILER_LAUNCHER": "ccache", "BUILD_TESTING": "ON" } },
    { "name": "ci-arm64", "inherits": "ci-amd64", "displayName": "CI arm64 (CPU tests only)",
      "cacheVariables": { "LAGRANGE_GPU_TESTS": "OFF" } },
    { "name": "ci-tidy", "inherits": "base", "displayName": "CI clang-tidy",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "Debug", "CMAKE_CXX_COMPILER": "clang++", "CMAKE_CXX_CLANG_TIDY": "clang-tidy" } }
  ],
  "buildPresets": [
    { "name": "dev", "configurePreset": "dev" },
    { "name": "release", "configurePreset": "release" },
    { "name": "asan", "configurePreset": "asan" },
    { "name": "tsan", "configurePreset": "tsan" }
  ],
  "testPresets": [
    { "name": "dev", "configurePreset": "dev", "output": { "outputOnFailure": true } },
    { "name": "asan", "configurePreset": "asan", "output": { "outputOnFailure": true },
      "environment": { "ASAN_OPTIONS": "detect_leaks=1:strict_string_checks=1", "UBSAN_OPTIONS": "halt_on_error=1:print_stacktrace=1" } },
    { "name": "tsan", "configurePreset": "tsan", "output": { "outputOnFailure": true },
      "environment": { "TSAN_OPTIONS": "halt_on_error=1:second_deadlock_stack=1" } }
  ]
}
```

Preset schema version 6 needs CMake 3.25 or newer; Resolute has 4.2.3. `cmakeMinimumRequired`
of 3.28 is a proposal to be confirmed against what Lyrical itself requires (TODO(verify)).

### `colcon.defaults.yaml`

```yaml
build:
  symlink-install: true
  cmake-args:
    - -DCMAKE_BUILD_TYPE=RelWithDebInfo
    - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    - -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
    - -DLAGRANGE_WARNINGS_AS_ERRORS=ON
  event-handlers: [console_cohesion+]
test:
  event-handlers: [console_direct+]
  return-code-on-test-failure: true
```

Sanitizer builds: `colcon build --build-base build-asan --install-base install-asan
--cmake-args -DLAGRANGE_SANITIZER="address;undefined"`. CLAUDE.md §7 mentions `--mixin asan`;
colcon mixins need the mixin repository to be registered on each machine, so the plan passes the
option explicitly (availability of mixins on Lyrical stays TODO(verify)).

## 5. Pre-commit

```yaml
# .pre-commit-config.yaml
# Every rev below is TODO(verify): resolve to an exact tag AND commit SHA when creating the file.
# Versions seen on 2026-10-13: clang-format 21.1.6, gersemi 0.29.2, ruff 0.17.0, mdformat 1.0.0,
# commitizen 4.19.2, buf 1.73.0, yamllint 1.38.0, reuse 6.2.0.
default_install_hook_types: [pre-commit, commit-msg]
minimum_pre_commit_version: "4.0.0"
repos:
  - repo: https://github.com/pre-commit/mirrors-clang-format
    rev: TODO(verify)          # a v21.x tag
    hooks:
      - id: clang-format
        types_or: [c++, c]
      - id: clang-format
        name: clang-format (slang)
        files: \.slang$
        types: [text]
  - repo: https://github.com/BlankSpruce/gersemi
    rev: TODO(verify)
    hooks: [{ id: gersemi }]
  - repo: https://github.com/astral-sh/ruff-pre-commit
    rev: TODO(verify)
    hooks:
      - id: ruff-check         # TODO(verify): hook id (`ruff` in older revisions, `ruff-check` in newer)
        args: [--fix]
      - id: ruff-format
  - repo: https://github.com/pre-commit/mirrors-mypy
    rev: TODO(verify)
    hooks:
      - id: mypy
        files: ^(python|mock_server|tools)/
        args: [--strict]
  - repo: https://github.com/bufbuild/buf
    rev: TODO(verify)
    hooks:
      - id: buf-format
      - id: buf-lint
  - repo: https://github.com/adrienverge/yamllint
    rev: TODO(verify)
    hooks: [{ id: yamllint, args: [--strict] }]
  - repo: https://github.com/hukkin/mdformat
    rev: TODO(verify)
    hooks:
      - id: mdformat
        args: [--wrap, "100"]
        additional_dependencies: [mdformat-gfm]     # TODO(verify) plugin name and version; tables
  - repo: https://github.com/scop/pre-commit-shfmt
    rev: TODO(verify)
    hooks: [{ id: shfmt, args: [-i, "4", -ci] }]
  - repo: https://github.com/shellcheck-py/shellcheck-py
    rev: TODO(verify)
    hooks: [{ id: shellcheck }]
  - repo: https://github.com/crate-ci/typos
    rev: TODO(verify)
    hooks: [{ id: typos }]
  - repo: https://github.com/fsfe/reuse-tool
    rev: TODO(verify)
    hooks: [{ id: reuse }]
  - repo: https://github.com/commitizen-tools/commitizen
    rev: TODO(verify)
    hooks:
      - id: commitizen
        stages: [commit-msg]
  - repo: https://github.com/python-jsonschema/check-jsonschema
    rev: TODO(verify)
    hooks:
      - id: check-jsonschema
        name: validate example profiles
        files: ^profiles/examples/.*\.ya?ml$
        args: [--schemafile, profiles/schema.json]
      - id: check-github-workflows
      - id: check-renovate
  - repo: https://github.com/pre-commit/pre-commit-hooks
    rev: TODO(verify)
    hooks:
      - id: trailing-whitespace
        exclude: \.md$
      - id: end-of-file-fixer
      - id: check-added-large-files   # blocks accidental big assets
        args: [--maxkb=1024]
      - id: check-merge-conflict
      - id: detect-private-key
      - id: check-yaml
        args: [--allow-multiple-documents]
      - id: check-json
      - id: check-toml
      - id: check-executables-have-shebangs
      - id: mixed-line-ending
        args: [--fix=lf]
  - repo: local
    hooks:
      - id: package-graph
        name: package dependency rules
        entry: python3 tools/check_package_graph.py
        language: system
        files: package\.xml$
        pass_filenames: false
      - id: no-float-positions
        name: world positions must be double
        entry: python3 tools/check_position_types.py
        language: system
        types_or: [c++]
```

No hook needs Node. `mdformat` replaces markdownlint. `.usda` files are not auto-formatted; they
are checked by loading them in tests.

### `pyproject.toml` (tooling sections)

```toml
[tool.ruff]
line-length = 100
target-version = "py314"          # Ubuntu 26.04 system Python; TODO(verify) ruff accepts py314
extend-exclude = ["proto/gen", "schemas/generated", "build*", "install*"]

[tool.ruff.lint]
select = ["E", "F", "W", "I", "B", "UP", "N", "SIM", "PTH", "RUF", "ANN", "D"]
ignore = ["D203", "D213"]

[tool.ruff.lint.pydocstyle]
convention = "google"

[tool.mypy]
python_version = "3.14"
strict = true
warn_unused_ignores = true
exclude = ["proto/gen", "build", "install"]

[tool.pytest.ini_options]
testpaths = ["python/tests", "mock_server/tests", "tools/tests"]
addopts = "-ra --strict-markers"
markers = ["gpu: needs a real GPU", "slow: nightly only"]

[tool.commitizen]
name = "cz_customize"
tag_format = "v$version"
version_provider = "scm"

[tool.commitizen.customize]
schema_pattern = '^(feat|fix|docs|ci|chore|refactor|perf|test|build|revert)(\((core|gpu|mod/[a-z_]+|viewer|proto|bridge/[a-z0-9_]+|io|schemas|profiles|server|mock|python|docs|ci|packaging|deps)\))?!?: [^A-Z].{0,70}[^.]$'
schema = "<type>(<scope>): <subject>"
example = "feat(viewer): add waveform scope panel"
```

### `buf.yaml` and `buf.gen.yaml`

```yaml
# buf.yaml
version: v2
modules:
  - path: proto
lint:
  use: [STANDARD]
  except: [RPC_REQUEST_STANDARD_NAME, RPC_RESPONSE_STANDARD_NAME, SERVICE_SUFFIX]
  enum_zero_value_suffix: _UNSPECIFIED
breaking:
  use: [WIRE_JSON]          # FILE from v1.0
```

```yaml
# buf.gen.yaml — used for docs and Python stubs; C++ code is generated by CMake with protoc 3.21
version: v2
plugins:
  - protoc_builtin: python
    out: proto/gen/python
  - protoc_builtin: pyi
    out: proto/gen/python
```

## 6. Commit message checks

Chosen: commitizen (configuration in `pyproject.toml` above), run as a `commit-msg` hook locally
and as `cz check` on the PR title and commit range in CI.

Alternative if the decision in Q13 goes the other way (Node only in CI, never locally):

```js
// commitlint.config.mjs — alternative, NOT the recommended option
export default {
  extends: ["@commitlint/config-conventional"],
  rules: {
    "type-enum": [2, "always",
      ["feat", "fix", "docs", "ci", "chore", "refactor", "perf", "test", "build", "revert"]],
    "scope-enum": [2, "always", [
      "core", "gpu", "viewer", "proto", "io", "schemas", "profiles", "server", "mock",
      "python", "docs", "ci", "packaging", "deps",
      "mod/rigid_body", "mod/electrical", "mod/thermal", "mod/aerodynamics", "mod/rotor_aero",
      "mod/weather", "mod/emi", "mod/radiation", "mod/sensors", "mod/vehicle_dynamics",
      "mod/hydrodynamics", "mod/traffic", "mod/comms",
      "bridge/ros2", "bridge/ros2_control", "bridge/mavlink", "bridge/autoware", "bridge/grpc",
      "bridge/zenoh"]],
    "subject-case": [2, "never", ["sentence-case", "start-case", "pascal-case", "upper-case"]],
    "header-max-length": [2, "always", 100],
    "signed-off-by": [2, "always", "Signed-off-by:"]
  }
};
```

DCO: the DCO GitHub App, or a small workflow step that checks every commit in the PR for a
`Signed-off-by:` trailer matching the author (shown in `ci.yml`). The workflow step is preferred
because it needs no third-party app installation.

## 7. `.github` files

### `CODEOWNERS`

```
# Later rules override earlier ones. Team names are placeholders until the teams are created in the organisation.
*                         @Open-Physical-Systems/maintainers

/core/                    @Open-Physical-Systems/core-maintainers
/gpu/                     @Open-Physical-Systems/gpu-maintainers
/viewer/                  @Open-Physical-Systems/viewer-maintainers
/modules/                 @Open-Physical-Systems/domain-maintainers
/modules/electrical/      @Open-Physical-Systems/electrical-maintainers
/modules/thermal/         @Open-Physical-Systems/electrical-maintainers
/bridges/                 @Open-Physical-Systems/ros-maintainers
/io/                      @Open-Physical-Systems/io-maintainers
/packaging/               @Open-Physical-Systems/release-maintainers
/vendor/                  @Open-Physical-Systems/release-maintainers
/docs/                    @Open-Physical-Systems/maintainers

# Stable surfaces: core-maintainer review is mandatory (CLAUDE.md §9); two approvals enforced by CI.
/schemas/                 @Open-Physical-Systems/core-maintainers
/core/plugin_abi/         @Open-Physical-Systems/core-maintainers
/proto/                   @Open-Physical-Systems/core-maintainers
/profiles/schema.json     @Open-Physical-Systems/core-maintainers
/rfcs/                    @Open-Physical-Systems/core-maintainers

# Supply chain and governance
/.github/                 @Open-Physical-Systems/release-maintainers @Open-Physical-Systems/core-maintainers
/LICENSE                  @Open-Physical-Systems/core-maintainers
/LICENSES/                @Open-Physical-Systems/core-maintainers
/REUSE.toml               @Open-Physical-Systems/core-maintainers
/SECURITY.md              @Open-Physical-Systems/core-maintainers
/GOVERNANCE.md            @Open-Physical-Systems/core-maintainers
/CLAUDE.md                @Open-Physical-Systems/core-maintainers
```

### `labels.yml`

```yaml
# Synced by .github/workflows/labels.yml
- { name: "type/bug",        color: "d73a4a", description: "Something is wrong" }
- { name: "type/feature",    color: "0e8a16", description: "New capability" }
- { name: "type/docs",       color: "0075ca", description: "Documentation" }
- { name: "type/test",       color: "bfd4f2", description: "Tests and validation" }
- { name: "type/perf",       color: "fbca04", description: "Performance" }
- { name: "type/chore",      color: "cfd3d7", description: "Maintenance, tooling" }
- { name: "type/spike",      color: "c5def5", description: "Time-boxed investigation with a written result" }
- { name: "type/security",   color: "b60205", description: "Security" }
- { name: "type/release",    color: "5319e7", description: "Release work" }
- { name: "type/validation", color: "1d76db", description: "Sim-vs-real or reference comparison" }

- { name: "area/core",        color: "5ebeff", description: "core/: scheduler, signals, world" }
- { name: "area/plugin-abi",  color: "5ebeff", description: "core/plugin_abi" }
- { name: "area/gpu",         color: "5ebeff", description: "gpu/: HAL, kernels, capability" }
- { name: "area/viewer",      color: "5ebeff", description: "viewer/" }
- { name: "area/proto",       color: "5ebeff", description: "proto/: UI-server protocol" }
- { name: "area/schemas",     color: "5ebeff", description: "schemas/: USD schemas" }
- { name: "area/profiles",    color: "5ebeff", description: "profiles/" }
- { name: "area/server",      color: "5ebeff", description: "server/" }
- { name: "area/mock",        color: "5ebeff", description: "mock_server/" }
- { name: "area/io",          color: "5ebeff", description: "io/: importers, recording, fields, geospatial" }
- { name: "area/python",      color: "5ebeff", description: "python/ bindings" }
- { name: "area/bridge-ros2", color: "5ebeff", description: "bridges/ros2" }
- { name: "area/bridge-ros2-control", color: "5ebeff", description: "bridges/ros2_control" }
- { name: "area/bridge-mavlink", color: "5ebeff", description: "bridges/mavlink" }
- { name: "area/bridge-autoware", color: "5ebeff", description: "bridges/autoware" }
- { name: "area/mod-rigid-body", color: "5ebeff", description: "modules/rigid_body" }
- { name: "area/mod-electrical", color: "5ebeff", description: "modules/electrical" }
- { name: "area/mod-thermal",    color: "5ebeff", description: "modules/thermal" }
- { name: "area/mod-aero",       color: "5ebeff", description: "modules/aerodynamics, rotor_aero" }
- { name: "area/mod-weather",    color: "5ebeff", description: "modules/weather" }
- { name: "area/mod-emi",        color: "5ebeff", description: "modules/emi" }
- { name: "area/mod-radiation",  color: "5ebeff", description: "modules/radiation" }
- { name: "area/mod-sensors",    color: "5ebeff", description: "modules/sensors" }
- { name: "area/mod-vehicle",    color: "5ebeff", description: "modules/vehicle_dynamics" }
- { name: "area/mod-traffic",    color: "5ebeff", description: "modules/traffic" }
- { name: "area/mod-comms",      color: "5ebeff", description: "modules/comms" }
- { name: "area/assets",      color: "5ebeff", description: "Sample robots and worlds" }
- { name: "area/demos",       color: "5ebeff", description: "Demo launch files and tutorials" }
- { name: "area/packaging",   color: "5ebeff", description: "Vendor packages, debs, Docker" }
- { name: "area/ci",          color: "5ebeff", description: "CI, tooling, repository" }
- { name: "area/docs",        color: "5ebeff", description: "docs/" }

- { name: "tier/0", color: "ededed", description: "CPU-only tier" }
- { name: "tier/1", color: "ededed", description: "Raster GPU tier" }
- { name: "tier/2", color: "ededed", description: "Full RT GPU tier" }

- { name: "needs-rfc",        color: "e99695", description: "Needs an accepted RFC before implementation" }
- { name: "needs-triage",     color: "e99695", description: "Not yet triaged" }
- { name: "needs-repro",      color: "e99695", description: "Cannot reproduce yet" }
- { name: "proto-break",      color: "b60205", description: "Intentional protocol break; 2 approvals" }
- { name: "schema-break",     color: "b60205", description: "Schema change needing migration; 2 approvals" }
- { name: "abi-break",        color: "b60205", description: "Plugin ABI break; 2 approvals" }
- { name: "good-first-issue", color: "7057ff", description: "Good for newcomers" }
- { name: "help-wanted",      color: "008672", description: "Extra attention is needed" }
- { name: "blocked",          color: "000000", description: "Blocked by another issue or upstream" }
- { name: "upstream",         color: "d4c5f9", description: "Depends on a third-party project" }
- { name: "backport/1.x",     color: "c2e0c6", description: "Cherry-pick to release/1.x" }
- { name: "dependencies",     color: "0366d6", description: "Dependency update (Renovate)" }
```

### `renovate.json`

```json
{
  "$schema": "https://docs.renovatebot.com/renovate-schema.json",
  "extends": ["config:recommended", ":semanticCommits", "helpers:pinGitHubActionDigests"],
  "timezone": "Asia/Tokyo",
  "schedule": ["before 6am on monday"],
  "labels": ["dependencies"],
  "semanticCommitType": "chore",
  "semanticCommitScope": "deps",
  "prConcurrentLimit": 5,
  "dependencyDashboard": true,
  "enabledManagers": ["github-actions", "pre-commit", "dockerfile", "pep621", "custom.regex"],
  "pre-commit": { "enabled": true },
  "packageRules": [
    { "matchManagers": ["github-actions"], "groupName": "github actions", "pinDigests": true },
    { "matchManagers": ["pre-commit"], "groupName": "pre-commit hooks" },
    { "matchManagers": ["dockerfile"], "groupName": "docker base images", "pinDigests": true },
    { "matchManagers": ["custom.regex"], "matchFileNames": ["vendor/**"], "groupName": "vendored sources",
      "labels": ["dependencies", "area/packaging"], "automerge": false },
    { "matchPackageNames": ["mirrors-clang-format"], "allowedVersions": "/^v21\\./",
      "description": "clang-format major is pinned; bump on purpose with a reformat PR" },
    { "matchUpdateTypes": ["major"], "dependencyDashboardApproval": true }
  ],
  "customManagers": [
    {
      "customType": "regex",
      "description": "Vendor package pins: `# renovate: datasource=github-tags depName=owner/repo` above `set(<X>_VERSION \"tag\")`",
      "managerFilePatterns": ["/^vendor/.+/CMakeLists\\.txt$/"],
      "matchStrings": [
        "# renovate: datasource=(?<datasource>[a-z-]+) depName=(?<depName>[^\\s]+)\\s+set\\([A-Z0-9_]+_VERSION \"(?<currentValue>[^\"]+)\"\\)"
      ]
    }
  ]
}
```

The timezone is a placeholder to be set by the maintainers. Option names follow Renovate's
documentation as remembered; the file is validated by the `check-renovate` pre-commit hook when
it is created (TODO(verify) `managerFilePatterns` vs the older `fileMatch` for the Renovate
version in use).

### Issue templates

```yaml
# .github/ISSUE_TEMPLATE/bug.yml
name: Bug report
description: Something does not work as documented
labels: ["type/bug", "needs-triage"]
body:
  - type: textarea
    id: what
    attributes: { label: What happened, description: "What you did, what you expected, what you saw." }
    validations: { required: true }
  - type: textarea
    id: repro
    attributes: { label: Steps to reproduce, description: "Profile YAML, stage file, commands. Attach a small MCAP if you can." }
    validations: { required: true }
  - type: input
    id: version
    attributes: { label: lagrange version, description: "Output of `lagrange info --version` or the git commit." }
    validations: { required: true }
  - type: dropdown
    id: install
    attributes: { label: Install method, options: ["apt (ros-lyrical-*)", "source (colcon)", "Docker image", "pip wheel"] }
    validations: { required: true }
  - type: input
    id: os
    attributes: { label: OS, description: "Output of `lsb_release -ds` and `uname -r`.", placeholder: "Ubuntu 26.04.1 LTS, 7.0.0-34-generic" }
    validations: { required: true }
  - type: dropdown
    id: arch
    attributes: { label: Architecture, options: [amd64, arm64] }
    validations: { required: true }
  - type: input
    id: gpu
    attributes: { label: GPU, description: "Model. Write 'none' for CPU-only.", placeholder: "NVIDIA RTX 4060" }
    validations: { required: true }
  - type: input
    id: driver
    attributes: { label: GPU driver, description: "`nvidia-smi` driver version or Mesa version.", placeholder: "NVIDIA 595.91.07" }
    validations: { required: true }
  - type: dropdown
    id: session
    attributes: { label: Display session, options: [Wayland, X11, "headless / none"] }
    validations: { required: true }
  - type: input
    id: tier
    attributes: { label: Capability tier, description: "From `lagrange_gpu_info` (0, 1 or 2)." }
  - type: input
    id: rmw
    attributes: { label: RMW implementation, placeholder: "rmw_fastrtps_cpp" }
  - type: textarea
    id: logs
    attributes: { label: Logs, render: text }
```

```yaml
# .github/ISSUE_TEMPLATE/feature.yml
name: Feature request
description: Propose a capability
labels: ["type/feature", "needs-triage"]
body:
  - type: textarea
    id: problem
    attributes: { label: Problem, description: "What are you trying to do that you cannot do now?" }
    validations: { required: true }
  - type: textarea
    id: proposal
    attributes: { label: Proposal }
    validations: { required: true }
  - type: dropdown
    id: area
    attributes:
      label: Area
      options: [viewer, core, a physics domain, sensors, ROS 2 integration, importers, protocol, packaging, docs, other]
    validations: { required: true }
  - type: checkboxes
    id: surfaces
    attributes:
      label: Does it change a stable surface?
      description: "These need an RFC (see rfcs/README.md)."
      options:
        - label: USD schemas
        - label: UI-server protocol
        - label: Plugin C ABI
        - label: Profile format
  - type: textarea
    id: validation
    attributes: { label: Validation, description: "For a physical model: which reference data or publication would it be checked against?" }
  - type: textarea
    id: tiers
    attributes: { label: Tier behaviour, description: "Expected behaviour on Tier 0 (CPU), Tier 1 (raster GPU), Tier 2 (RT GPU)." }
```

```yaml
# .github/ISSUE_TEMPLATE/robot-class.yml
name: Robot class or integration request
description: Ask for support for a kind of robot or an external stack
labels: ["type/feature", "needs-triage"]
body:
  - type: input
    id: robot
    attributes: { label: Robot class or stack, placeholder: "Tracked vehicle; Open-RMF fleet adapter; ..." }
    validations: { required: true }
  - type: textarea
    id: usecase
    attributes: { label: Use case }
    validations: { required: true }
  - type: textarea
    id: domains
    attributes: { label: Domains needed, description: "Rigid body, electrical, thermal, aerodynamics, weather, EMI, sensors, ..." }
  - type: textarea
    id: integrations
    attributes: { label: Integrations, description: "ROS 2 packages, controllers, autopilots; state their Lyrical availability if you know it." }
  - type: textarea
    id: model
    attributes: { label: Model availability, description: "Is there a URDF/SDF/MJCF/USD you can share, and under which license?" }
```

```yaml
# .github/ISSUE_TEMPLATE/config.yml
blank_issues_enabled: false
contact_links:
  - name: Question or discussion
    url: https://github.com/Open-Physical-Systems/Lagrange/discussions     
    about: Ask usage questions here, not in issues.
  - name: Security vulnerability
    url: https://github.com/Open-Physical-Systems/Lagrange/security/advisories/new
    about: Report privately. Do not open a public issue.
```

### `PULL_REQUEST_TEMPLATE.md`

```markdown
<!-- Title must be a Conventional Commit: type(scope): subject   e.g. feat(viewer): add scope cursors -->

## What and why

## How it was tested
<!-- Commands, new tests, screenshots for viewer changes. -->

## Checklist
- [ ] Commits are signed off (DCO).
- [ ] `pre-commit run --all-files` passes.
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
```

## 8. Workflows

Common rules: every `uses:` is pinned to a full commit SHA with the version in a comment;
top-level `permissions: {}` with per-job grants; `concurrency` cancels superseded PR runs;
containers are `ros:lyrical-ros-base-resolute` (tag confirmed on Docker Hub 2026-10-13, amd64 and
arm64), referenced by digest once created.

### `ci.yml`

```yaml
name: ci
on:
  pull_request:
  merge_group:
  push:
    branches: [main]
permissions: {}
concurrency:
  group: ci-${{ github.workflow }}-${{ github.event.pull_request.number || github.ref }}
  cancel-in-progress: ${{ github.event_name == 'pull_request' }}

env:
  ROS_DISTRO: lyrical
  CCACHE_DIR: ${{ github.workspace }}/.ccache
  CCACHE_MAXSIZE: 2G

jobs:
  lint:
    runs-on: ubuntu-latest            # TODO(verify) runner label; job runs inside the container anyway
    container: ros:lyrical-ros-base-resolute   # TODO(verify): pin by digest
    permissions: { contents: read }
    steps:
      - uses: actions/checkout@TODO_VERIFY_SHA          # TODO(verify) version + SHA
        with: { fetch-depth: 0 }
      - name: Install tools
        run: |
          apt-get update
          apt-get install -y --no-install-recommends \
            pre-commit clang-format clang-tidy shellcheck shfmt yamllint mdformat reuse python3-pip git
      - uses: actions/cache@TODO_VERIFY_SHA             # TODO(verify)
        with:
          path: ~/.cache/pre-commit
          key: pre-commit-${{ hashFiles('.pre-commit-config.yaml') }}
      - name: pre-commit (all hooks)
        run: |
          git config --global --add safe.directory "$GITHUB_WORKSPACE"
          pre-commit run --all-files --show-diff-on-failure

  commits:
    if: github.event_name == 'pull_request'
    runs-on: ubuntu-latest
    permissions: { contents: read, pull-requests: read }
    steps:
      - uses: actions/checkout@TODO_VERIFY_SHA
        with: { fetch-depth: 0 }
      - name: Install commitizen
        run: pipx install commitizen==TODO_VERIFY_VERSION   # 4.19.2 seen on PyPI
      - name: PR title is a Conventional Commit
        env: { PR_TITLE: "${{ github.event.pull_request.title }}" }
        run: cz check --message "$PR_TITLE"
      - name: DCO sign-off on every commit
        env:
          BASE: ${{ github.event.pull_request.base.sha }}
          HEAD: ${{ github.event.pull_request.head.sha }}
        run: |
          fail=0
          for c in $(git rev-list --no-merges "$BASE..$HEAD"); do
            author="$(git show -s --format='%an <%ae>' "$c")"
            if ! git show -s --format='%(trailers:key=Signed-off-by,valueonly)' "$c" | grep -Fxq "$author"; then
              echo "::error::commit $c is not signed off by its author ($author)"; fail=1
            fi
          done
          exit "$fail"

  reuse:
    runs-on: ubuntu-latest
    permissions: { contents: read }
    steps:
      - uses: actions/checkout@TODO_VERIFY_SHA
      - uses: fsfe/reuse-action@TODO_VERIFY_SHA         # TODO(verify)

  proto:
    runs-on: ubuntu-latest
    permissions: { contents: read, pull-requests: read }
    steps:
      - uses: actions/checkout@TODO_VERIFY_SHA
        with: { fetch-depth: 0 }
      - uses: bufbuild/buf-action@TODO_VERIFY_SHA       # TODO(verify); buf 1.73.0 seen
        with:
          lint: true
          format: true
          breaking: ${{ !contains(github.event.pull_request.labels.*.name, 'proto-break') }}
          breaking_against: "${{ github.event.repository.clone_url }}#branch=main"

  build-test:
    strategy:
      fail-fast: false
      matrix:
        include:
          - { arch: amd64, runner: ubuntu-latest,    preset: ci-amd64 }
          - { arch: arm64, runner: ubuntu-24.04-arm, preset: ci-arm64 }   # TODO(verify) arm64 runner label
    name: build-test (${{ matrix.arch }})
    runs-on: ${{ matrix.runner }}
    container: ros:lyrical-ros-base-resolute
    permissions: { contents: read }
    steps:
      - uses: actions/checkout@TODO_VERIFY_SHA
        with: { path: src/lagrange }
      - name: Restore ccache
        uses: actions/cache@TODO_VERIFY_SHA
        with:
          path: .ccache
          key: ccache-${{ matrix.arch }}-${{ github.sha }}
          restore-keys: ccache-${{ matrix.arch }}-
      - name: Restore vendor underlay (OpenUSD etc.)
        id: vendor
        uses: actions/cache@TODO_VERIFY_SHA
        with:
          path: vendor_install
          key: vendor-${{ matrix.arch }}-${{ hashFiles('src/lagrange/vendor/**') }}
      - name: Install dependencies
        run: |
          apt-get update
          apt-get install -y --no-install-recommends ccache ninja-build python3-colcon-common-extensions python3-rosdep
          cp src/lagrange/packaging/rosdep/lagrange.yaml /etc/ros/rosdep/sources.list.d/lagrange.yaml || true
          echo "yaml file:///etc/ros/rosdep/sources.list.d/lagrange.yaml" > /etc/ros/rosdep/sources.list.d/10-lagrange.list
          rosdep update
          rosdep install --from-paths src --ignore-src -y --rosdistro "$ROS_DISTRO"
      - name: Build vendor packages (cache miss only)
        if: steps.vendor.outputs.cache-hit != 'true'
        shell: bash
        run: |
          source /opt/ros/$ROS_DISTRO/setup.bash
          colcon build --base-paths src/lagrange/vendor --install-base vendor_install --merge-install \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
      - name: Build
        shell: bash
        run: |
          source /opt/ros/$ROS_DISTRO/setup.bash
          [ -f vendor_install/setup.bash ] && source vendor_install/setup.bash
          colcon build --base-paths src/lagrange --packages-skip-regex '.*_vendor' \
            --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
                         -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DLAGRANGE_WARNINGS_AS_ERRORS=ON
      - name: Test (gtest + pytest via colcon)
        shell: bash
        run: |
          source install/setup.bash
          colcon test --packages-skip-regex '.*_vendor' --ctest-args -LE 'gpu|slow'
          colcon test-result --verbose
      - name: Upload test results
        if: always()
        uses: actions/upload-artifact@TODO_VERIFY_SHA
        with: { name: "test-results-${{ matrix.arch }}", path: "build/*/test_results/**" }

  sanitizers:
    strategy:
      fail-fast: false
      matrix:
        include:
          - { name: asan-ubsan, flags: "address;undefined" }
          - { name: tsan,       flags: "thread" }
    name: sanitizers (${{ matrix.name }})
    runs-on: ubuntu-latest
    container:
      image: ros:lyrical-ros-base-resolute
      options: --cap-add=SYS_PTRACE --security-opt seccomp=unconfined   # TSan needs ASLR control
    permissions: { contents: read }
    steps:
      - uses: actions/checkout@TODO_VERIFY_SHA
        with: { path: src/lagrange }
      # ... same dependency and vendor-cache steps as build-test (shared through a composite action
      #     .github/actions/setup-workspace) ...
      - name: Build and test with sanitizer
        shell: bash
        run: |
          source /opt/ros/$ROS_DISTRO/setup.bash
          colcon build --base-paths src/lagrange --packages-skip-regex '.*_vendor' \
            --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON "-DLAGRANGE_SANITIZER=${{ matrix.flags }}"
          source install/setup.bash
          colcon test --packages-skip-regex '.*_vendor' --ctest-args -LE 'gpu|slow'
          colcon test-result --verbose

  viewer-headless:
    runs-on: ubuntu-latest
    container: ros:lyrical-ros-base-resolute
    permissions: { contents: read }
    steps:
      - uses: actions/checkout@TODO_VERIFY_SHA
        with: { path: src/lagrange }
      - name: Install software Vulkan
        run: |
          apt-get update
          apt-get install -y --no-install-recommends mesa-vulkan-drivers vulkan-tools libvulkan1
      # ... workspace setup and build of lagrange_proto, lagrange_gpu_*, lagrange_mock_server, lagrange_viewer ...
      - name: Headless viewer against the mock server (lavapipe)
        shell: bash
        env:
          VK_ICD_FILENAMES: /usr/share/vulkan/icd.d/lvp_icd.json    # TODO(verify) arch-suffixed name in the image
          SDL_VIDEODRIVER: offscreen                                 # TODO(verify) SDL3 driver for surfaceless Vulkan
          LAGRANGE_TIER: "0"
        run: |
          source install/setup.bash
          vulkaninfo --summary
          colcon test --packages-select lagrange_viewer --ctest-args -L headless
          colcon test-result --verbose
      - name: Upload images on failure
        if: failure()
        uses: actions/upload-artifact@TODO_VERIFY_SHA
        with: { name: viewer-golden-diff, path: "build/lagrange_viewer/test_output/**" }

  docs:
    runs-on: ubuntu-latest
    permissions: { contents: read }
    steps:
      - uses: actions/checkout@TODO_VERIFY_SHA
      - name: Build docs and check links
        run: |
          pipx run --spec "TODO_VERIFY_DOCS_TOOL" true    # Sphinx or MkDocs, decided in issue 0.1-A9
          ./tools/build_docs.sh --strict --linkcheck

  two-approvals-on-stable-surfaces:
    if: github.event_name == 'pull_request'
    runs-on: ubuntu-latest
    permissions: { contents: read, pull-requests: read }
    steps:
      - uses: actions/checkout@TODO_VERIFY_SHA
        with: { fetch-depth: 0 }
      - name: Require 2 approvals when schemas/, core/plugin_abi/ or proto/ change
        env:
          GH_TOKEN: ${{ github.token }}
          PR: ${{ github.event.pull_request.number }}
          BASE: ${{ github.event.pull_request.base.sha }}
          HEAD: ${{ github.event.pull_request.head.sha }}
        run: |
          if git diff --name-only "$BASE...$HEAD" | grep -Eq '^(schemas/|core/plugin_abi/|proto/|profiles/schema\.json)'; then
            n=$(gh pr view "$PR" --json reviews --jq '[.reviews[] | select(.state=="APPROVED") | .author.login] | unique | length')
            [ "$n" -ge 2 ] || { echo "::error::stable-surface change needs 2 approvals, has $n"; exit 1; }
          fi
```

The approval check must re-run when a review is submitted (`pull_request_review` trigger in a
small companion workflow); otherwise it stays red after the second approval.

### `merge-queue.yml`

```yaml
name: merge-queue
on:
  merge_group:
permissions: {}
jobs:
  gpu-tests:
    runs-on: [self-hosted, linux, x64, gpu-rtx]      # hardened runner; never runs fork PRs directly
    permissions: { contents: read }
    steps:
      - uses: actions/checkout@TODO_VERIFY_SHA
        with: { path: src/lagrange }
      - name: Build and run GPU-labelled tests in a container with the NVIDIA runtime
        run: ./src/lagrange/tools/ci/run_in_container.sh --gpus all -- colcon test --ctest-args -L gpu
  determinism:
    runs-on: ubuntu-latest
    container: ros:lyrical-ros-base-resolute
    permissions: { contents: read }
    steps:
      - uses: actions/checkout@TODO_VERIFY_SHA
        with: { path: src/lagrange }
      - name: Same scenario twice, bit-identical CPU output
        run: ./src/lagrange/tools/ci/determinism.sh profiles/examples/determinism_amr.yaml --sim-seconds 10
```

### `nightly.yml` (outline)

| Job | Runner | Content |
|---|---|---|
| `validation` | hosted + GPU runner | Sim-vs-real and reference tests labelled `validation`; results published as a report artifact |
| `performance` | GPU runner | Viewer `--benchmark`, scheduler and sensor benchmarks; compare with stored baselines; fail above threshold (10 % proposed) |
| `rmw-matrix` | hosted | ROS integration tests with `rmw_fastrtps_cpp` and `rmw_zenoh_cpp` |
| `weekly-clean-build` (Mondays) | hosted, both architectures | No caches; catches apt archive drift and vendor build breakage |
| `fuzz` | hosted | Protocol frame reader and parser fuzzers for 30 minutes |
| `demos` | GPU runner | Nav2 demo (v0.2+), later MoveIt and SITL demos |

### `release.yml`

```yaml
name: release
on:
  push:
    branches: [main, "release/*"]
permissions: {}
jobs:
  release-please:
    runs-on: ubuntu-latest
    permissions: { contents: write, pull-requests: write }
    outputs:
      created: ${{ steps.rp.outputs.release_created }}
      tag: ${{ steps.rp.outputs.tag_name }}
    steps:
      - id: rp
        uses: googleapis/release-please-action@TODO_VERIFY_SHA     # TODO(verify); release-please 17.11.2 seen
        with:
          config-file: release-please-config.json
          manifest-file: .release-please-manifest.json

  artifacts:
    needs: release-please
    if: needs.release-please.outputs.created == 'true'
    strategy:
      matrix:
        include:
          - { arch: amd64, runner: ubuntu-latest }
          - { arch: arm64, runner: ubuntu-24.04-arm }      # TODO(verify)
    runs-on: ${{ matrix.runner }}
    permissions: { contents: write, id-token: write, attestations: write, packages: write }
    steps:
      - uses: actions/checkout@TODO_VERIFY_SHA
        with: { ref: "${{ needs.release-please.outputs.tag }}" }
      - name: Build source tarball, vendor underlay tarball, pip wheel
        run: ./tools/release/build_artifacts.sh --arch "${{ matrix.arch }}"
      - name: Build Docker image (ubuntu:resolute base)
        run: ./tools/release/build_image.sh --arch "${{ matrix.arch }}" --tag "${{ needs.release-please.outputs.tag }}"
      - name: SBOM (SPDX)
        uses: anchore/sbom-action@TODO_VERIFY_SHA                 # TODO(verify) tool choice
        with: { path: ., format: spdx-json, output-file: "sbom-${{ matrix.arch }}.spdx.json" }
      - name: Sign artifacts (Sigstore, keyless)
        uses: sigstore/gh-action-sigstore-python@TODO_VERIFY_SHA  # TODO(verify)
        with: { inputs: "dist/*" }
      - name: Build provenance attestation
        uses: actions/attest-build-provenance@TODO_VERIFY_SHA     # TODO(verify)
        with: { subject-path: "dist/*" }
      - name: Upload to the GitHub release
        env: { GH_TOKEN: "${{ github.token }}" }
        run: gh release upload "${{ needs.release-please.outputs.tag }}" dist/* sbom-*.spdx.json

  debs:
    needs: [release-please, artifacts]
    if: needs.release-please.outputs.created == 'true'
    runs-on: ubuntu-latest
    permissions: { contents: read }
    steps:
      - name: Open the bloom release PR against ros/rosdistro (lyrical) or publish to the project apt repository
        run: echo "Decided by open question Q18; manual step until v1.0"
```

```json
// release-please-config.json
{
  "$schema": "https://raw.githubusercontent.com/googleapis/release-please/main/schemas/config.json",
  "release-type": "simple",
  "include-component-in-tag": false,
  "bump-minor-pre-major": true,
  "packages": { ".": { "package-name": "lagrange", "changelog-path": "CHANGELOG.md",
    "extra-files": [{ "type": "xml", "path": "**/package.xml", "xpath": "//package/version" }] } },
  "changelog-sections": [
    { "type": "feat", "section": "Features" }, { "type": "fix", "section": "Bug fixes" },
    { "type": "perf", "section": "Performance" }, { "type": "docs", "section": "Documentation" },
    { "type": "refactor", "section": "Refactoring", "hidden": true },
    { "type": "chore", "section": "Chores", "hidden": true }, { "type": "ci", "section": "CI", "hidden": true }
  ]
}
```

All `package.xml` versions move together (one version for the repository), which is what bloom
expects for a multi-package repository. The `extra-files` glob and XML updater syntax are
TODO(verify) against release-please's documentation.

### Other workflows

| File | Content |
|---|---|
| `codeql.yml` | CodeQL for `c-cpp` (manual build mode using the colcon build) and `python`; weekly and on PRs touching code |
| `scorecard.yml` | OpenSSF Scorecard weekly, results to code scanning; publish the badge |
| `labels.yml` | Sync `.github/labels.yml` on change (action TODO(verify)) |
| `docs.yml` | Build and publish the docs site from `main` and from release tags |

## 9. CI job matrix

| Job | Trigger | Runner | Arch | Time target |
|---|---|---|---|---|
| `lint` | PR, queue | hosted, container | amd64 | 5 min |
| `commits` | PR | hosted | amd64 | 1 min |
| `reuse` | PR, queue | hosted | amd64 | 1 min |
| `proto` | PR, queue | hosted | amd64 | 2 min |
| `build-test` | PR, queue, main | hosted, container | amd64, arm64 | 20 min with caches |
| `sanitizers` | PR, queue | hosted, container | amd64 | 25 min |
| `viewer-headless` | PR, queue | hosted, container, lavapipe | amd64 | 15 min |
| `docs` | PR, queue | hosted | amd64 | 5 min |
| `gpu-tests` | merge queue, nightly | self-hosted RTX | amd64 | 20 min |
| `determinism` | merge queue | hosted, container | amd64 | 10 min |
| `validation`, `performance`, `demos`, `rmw-matrix`, `fuzz` | nightly | mixed | amd64 | — |
| `weekly-clean-build` | weekly | hosted | amd64, arm64 | up to 2 h |
| `codeql`, `scorecard` | weekly + PR | hosted | amd64 | — |
| `release` | push to `main` / `release/*` | hosted | amd64, arm64 | — |

GPU runner options (verification §10): GitHub-hosted GPU runners cost about USD 0.052 per minute
and need a Team or Enterprise plan; their GPU model is unverified. A self-hosted RTX machine is
needed regardless for Tier 2 (hardware ray tracing) tests and the performance budgets. Security
rule for the self-hosted runner on a public repository: it runs only `merge_group`, `schedule`
and `workflow_dispatch` events, never `pull_request` from forks, inside ephemeral containers.

## 10. Release and distribution

| Artifact | How | From |
|---|---|---|
| Source release | release-please PR → tag → GitHub release with changelog | v0.1 |
| Vendor underlay tarballs (amd64, arm64) | Built in the release workflow; lets contributors skip OpenUSD builds | v0.2 |
| Docker images | `ubuntu:resolute` base + ROS Lyrical + lagrange; `-runtime` and `-dev` variants; pushed to GHCR; NVIDIA runtime optional | v0.2 |
| pip wheel | Python bindings wheel for the system Python (3.14), `manylinux` tag TODO(verify) feasibility with the USD dependency | v0.3 |
| `ros-lyrical-lagrange-*` debs | bloom → ROS buildfarm if vendor packages build there; else a project apt repository built by this CI (Q18) | trial in v0.2, official at v1.0 |
| SBOM | SPDX JSON per release artifact | v0.1 |
| Signatures and provenance | Sigstore keyless signatures; GitHub build provenance attestations | v0.1 |

## 11. Security

| Item | Plan |
|---|---|
| `SECURITY.md` | Private reporting through GitHub security advisories; response target 7 days; supported versions table; scope note that the viewer protocol is unauthenticated before v1.0 and must not be exposed to untrusted networks |
| Secret scanning | Enabled with push protection |
| Code scanning | CodeQL; clang-tidy `bugprone-*` as errors; sanitizers; fuzzing of the protocol parser |
| Supply chain | Actions pinned to SHAs; Renovate weekly with digests; vendor sources pinned by tag + SHA-256; no network downloads at build time outside vendor packages; OpenSSF Scorecard |
| Runner isolation | Self-hosted GPU runner rules in section 9 |
| Dependency licenses | REUSE lint; SBOM diff reviewed at release |
| Protocol hardening | Frame size limit, bounded allocations from untrusted lengths, shared-memory generation checks, TCP bound to localhost by default |

## 12. `REUSE.toml` (sketch)

```toml
version = 1

[[annotations]]
path = ["**/*.cpp", "**/*.hpp", "**/*.h", "**/*.slang", "**/*.py", "**/*.proto", "**/CMakeLists.txt", "**/*.cmake"]
precedence = "closest"
SPDX-FileCopyrightText = "The lagrange authors"
SPDX-License-Identifier = "Apache-2.0"

[[annotations]]
path = ["docs/**", "rfcs/**", "*.md", "profiles/**", "schemas/**", "assets/**", ".github/**", "*.json", "*.yaml", "*.yml", "*.toml"]
precedence = "closest"
SPDX-FileCopyrightText = "The lagrange authors"
SPDX-License-Identifier = "Apache-2.0"

[[annotations]]
path = "viewer/assets/fonts/**"
precedence = "override"
SPDX-FileCopyrightText = "See the license file next to each font"
SPDX-License-Identifier = "OFL-1.1"
```

Whether documentation should be CC-BY-4.0 instead of Apache-2.0 is a small open point; one
license for everything is simpler and is the proposal.
