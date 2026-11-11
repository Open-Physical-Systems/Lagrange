#!/usr/bin/env bash
# Packs the debs build_debs.sh made into the archive packaging/install.sh downloads from a GitHub
# release: lagrange-VERSION-debs-amd64.tar (the debs and pip-constraints.txt, flat) and its
# .sha256. Usage (from the source tree): packaging/debs/release_archive.sh DEB_DIR OUT_DIR
# then: gh release upload vVERSION OUT_DIR/lagrange-VERSION-debs-amd64.tar{,.sha256}
set -euo pipefail

debs="$(realpath "${1:?usage: packaging/debs/release_archive.sh DEB_DIR OUT_DIR}")"
out="$(realpath -m "${2:?usage: packaging/debs/release_archive.sh DEB_DIR OUT_DIR}")"
version="$(sed -n 's:.*<version>\(.*\)</version>.*:\1:p' tools/cli/package.xml | head -1)"
[ -n "$version" ] || { echo "no version in tools/cli/package.xml" >&2; exit 1; }
ls "$debs"/ros-lyrical-lagrange*_"$version"-*.deb >/dev/null 2>&1 ||
    { echo "no ros-lyrical-lagrange* $version debs in $debs" >&2; exit 1; }
mkdir -p "$out"
archive="lagrange-$version-debs-amd64.tar"
stage="$(mktemp -d)"
trap 'rm -rf "$stage"' EXIT
cp "$debs"/ros-lyrical-*_"$version"-*.deb "$stage/"
cp packaging/pip-constraints.txt "$stage/"
# Sorted names and fixed owners and times, so the same debs give the same archive.
(cd "$stage" && find . -maxdepth 1 -type f -printf '%P\n' | LC_ALL=C sort |
    tar --owner=0 --group=0 --numeric-owner --mtime=@0 -cf "$out/$archive" -T -)
(cd "$out" && sha256sum "$archive" >"$archive.sha256")
echo "$out/$archive: $(tar -tf "$out/$archive" | grep -c '\.deb$') debs, $(du -h "$out/$archive" | cut -f1)"
