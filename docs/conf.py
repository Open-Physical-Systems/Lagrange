"""Sphinx settings for the documentation site (https://open-physical-systems.github.io/Lagrange).

The pages are the Markdown files in docs/ as they are; the site only adds the menu (index.md),
diagrams (```mermaid blocks) and math ($...$, $$...$$). Build it with tools/docs/build.sh; the
docs workflow publishes it. Links from a page to a file outside the site (source code, scripts,
the RFCs, docs/plan) are turned into links to that file on GitHub, so the same Markdown reads
right both on GitHub and on the site.
"""

import re
from pathlib import Path

from sphinx.application import Sphinx

DOCS = Path(__file__).resolve().parent
REPO = DOCS.parent
GITHUB = "https://github.com/Open-Physical-Systems/Lagrange"
BRANCH = "dev"

project = "Lagrange"
author = "Open Physical Systems"
copyright = "Open Physical Systems contributors, Apache-2.0"

extensions = ["myst_parser", "sphinxcontrib.mermaid"]
source_suffix = {".md": "markdown"}
root_doc = "index"
# The project's own planning notes and the viewer's screen specs stay on GitHub, not on the site.
PRIVATE = ("plan", "design/ui")
exclude_patterns = ["_build", *PRIVATE]

myst_enable_extensions = ["dollarmath", "amsmath", "colon_fence", "deflist"]
myst_heading_anchors = 4
myst_fence_as_directive = ["mermaid"]
suppress_warnings = ["myst.header"]  # some design notes skip heading levels

mermaid_version = "11.12.1"  # sphinxcontrib-mermaid 2.1.1's default, pinned here

html_theme = "furo"
html_title = "Lagrange"
html_baseurl = "https://open-physical-systems.github.io/Lagrange/"
html_theme_options = {
    "source_repository": f"{GITHUB}/",
    "source_branch": BRANCH,
    "source_directory": "docs/",
}

# [text](target) with a relative target; images too (![alt](target)).
_LINK = re.compile(r"(\]\()([^)\s]+)(\))")
_SITE_SUFFIXES = {".md", ".png", ".jpg", ".jpeg", ".gif", ".svg", ".mp4", ".webm"}


def _on_github(docname: str, target: str) -> str:
    """The link to use on the site for a link written in docs/<docname>.md."""
    if re.match(r"^[a-z][a-z0-9+.-]*:", target) or target.startswith(("#", "/")):
        return target
    path, _, anchor = target.partition("#")
    if not path:
        return target
    resolved = (DOCS / docname).parent.joinpath(path).resolve()
    try:
        inside = resolved.relative_to(DOCS)
    except ValueError:
        inside = None
    public = inside is not None and not any(inside.as_posix().startswith(p + "/") for p in PRIVATE)
    if public and resolved.suffix in _SITE_SUFFIXES:
        return target
    try:
        relative = resolved.relative_to(REPO).as_posix()
    except ValueError:
        return target
    kind = "tree" if resolved.is_dir() else "blob"
    return f"{GITHUB}/{kind}/{BRANCH}/{relative}" + (f"#{anchor}" if anchor else "")


def _rewrite_links(app: Sphinx, docname: str, source: list[str]) -> None:
    del app
    source[0] = _LINK.sub(lambda m: m[1] + _on_github(docname, m[2]) + m[3], source[0])


def setup(app: Sphinx) -> dict[str, bool]:
    app.connect("source-read", _rewrite_links)
    return {"parallel_read_safe": True}
