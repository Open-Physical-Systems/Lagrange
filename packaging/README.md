# Packaging

How a Lagrange release becomes installable (roadmap §9, "Binary releases"). Everything here
builds and checks the artefacts locally; publishing them is the maintainers' step (§Publishing).

| Artefact | Built by | Checked by |
|---|---|---|
| `ros-lyrical-lagrange-*` .debs (every package, bloom) | `packaging/debs/build_debs.sh OUT_DIR` (in a clean `ros:lyrical-ros-base-resolute` container, dependency order, each installed before the next) | `packaging/test_install.sh OUT_DIR` |
| Container image from the debs alone | `packaging/container/Containerfile.release` (no sources, no compiler) | `packaging/test_install.sh`: the clean-machine install test |
| Development image from sources | `packaging/container/Containerfile` | its default command (the Nav2 demo) |
| Python wheel | `packaging/wheel/build_wheel.sh PACKAGE_DIR OUT_DIR` | imports and runs a simulation; a platform wheel for Ubuntu 26.04 with the debs installed, not a self-contained one |
| SBOM (SPDX 2.3, JSON) | `packaging/sbom/make_sbom.py OUT.spdx.json` (our packages from `package.xml`, third parties from `docs/design/dependencies.md`) | well-formed, unique IDs |

```bash
packaging/debs/build_debs.sh ~/lagrange_debs        # about an hour on 8 cores
packaging/test_install.sh ~/lagrange_debs lagrange:1.0.0
packaging/wheel/build_wheel.sh install/lagrange_python/lib/python3.14/site-packages/lagrange dist/
python3 packaging/sbom/make_sbom.py dist/lagrange-1.0.0.spdx.json
```

The rosdep keys of our own packages (`lagrange_*` to `ros-lyrical-lagrange-*`) are generated in
the container from the `package.xml` files until the packages are in rosdistro;
`packaging/rosdep/lagrange.yaml` holds the third-party keys rosdistro lacks.

## Publishing (maintainers)

These need the project's accounts and identity, so they are not scripted here:

1. **Debs.** Either the ROS buildfarm (a `bloom-release` of each repository package into
   `ros/rosdistro`'s `lyrical/distribution.yaml`; the vendor packages for OpenUSD and Jolt are
   long builds the buildfarm accepts, `docs/plan/spikes/0.2-H8-packaging.md` §3), or an own apt
   repository of the debs `build_debs.sh` makes.
2. **Image.** Tag the image `test_install.sh` built and push it to the project's registry.
3. **Wheel.** Attach it to the GitHub release (it is not useful on PyPI without the debs).
4. **Signatures.** Sign every artefact and the SBOM with Sigstore (`cosign sign-blob` for files,
   `cosign sign` for the image) under the maintainers' identity, and attach the bundles to the
   release.
