"""Sphinx settings for the documentation site (https://open-physical-systems.github.io/Lagrange).

The pages are the Markdown files in docs/ as they are; the site only adds the menu (index.md),
diagrams (```mermaid blocks, drawn in the browser by _static/mermaid-init.js), math ($...$,
$$...$$) and players for videos (![caption](file.mp4)). Build it with tools/docs/build.sh; the
docs workflow publishes it. Links from a page to a file outside the site (source code, scripts,
the RFCs, docs/plan) are turned into links to that file on GitHub, so the same Markdown reads
right both on GitHub and on the site.
"""

import html
import re
import shutil
from pathlib import Path

from docutils import nodes
from sphinx.application import Sphinx
from sphinx.util.docutils import SphinxDirective

DOCS = Path(__file__).resolve().parent
REPO = DOCS.parent
GITHUB = "https://github.com/Open-Physical-Systems/Lagrange"
BRANCH = "main"  # the site is published from releases

project = "Lagrange"
author = "Open Physical Systems"
copyright = "Open Physical Systems contributors, Apache-2.0"

extensions = ["myst_parser"]
source_suffix = {".md": "markdown"}
root_doc = "index"
# The project's own planning notes and the viewer's screen specs stay on GitHub, not on the site.
PRIVATE = ("plan", "design/ui")
exclude_patterns = ["_build", *PRIVATE]

myst_enable_extensions = ["dollarmath", "amsmath", "colon_fence", "deflist"]
myst_heading_anchors = 4
myst_fence_as_directive = ["mermaid"]
suppress_warnings = ["myst.header"]  # some design notes skip heading levels

html_theme = "furo"
html_static_path = ["_static"]
html_logo = "imgs/repo-logo/lagrange-lockup@2x.png"
html_favicon = "imgs/repo-logo/lagrange-square-mark@2x.png"
html_css_files = ["site.css"]
# Strong colours for code, the same in the light and the dark theme.
pygments_style = "monokai"
pygments_dark_style = "monokai"
html_title = "Lagrange"
html_baseurl = "https://open-physical-systems.github.io/Lagrange/"
html_theme_options = {
    "source_repository": f"{GITHUB}/",
    "source_branch": BRANCH,
    "source_directory": "docs/",
    "sidebar_hide_name": True,  # the logo carries the name
    # The design tokens' accent (docs/design/ui/shared/design-tokens.json), light and dark.
    "light_css_variables": {"color-brand-primary": "#C9611D", "color-brand-content": "#B4561A"},
    "dark_css_variables": {"color-brand-primary": "#E8772E", "color-brand-content": "#F0904D"},
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


# ![caption](file.mp4), and on the video page every [name](file.mp4): a player instead.
_VIDEO_IMAGE = re.compile(r"!\[([^\]]*)\]\(([^)\s]+\.mp4)\)")
_VIDEO_LINK = re.compile(r"\[([^\]]*)\]\(([^)\s:]+\.mp4)\)")


def _player(src: str, caption: str, *, block: bool) -> str:
    video = (
        f'<video class="doc-video" src="{html.escape(src)}" title="{html.escape(caption)}" '
        'controls muted loop playsinline preload="metadata"></video>'
    )
    if not block:
        return video
    return f'\n\n{video}\n<p class="doc-video-caption">{html.escape(caption)}</p>\n\n'


def _rewrite_links(app: Sphinx, docname: str, source: list[str]) -> None:
    del app
    text = _LINK.sub(lambda m: m[1] + _on_github(docname, m[2]) + m[3], source[0])
    text = _VIDEO_IMAGE.sub(lambda m: _player(m[2], m[1], block=True), text)
    if docname == "videos/README":
        text = _VIDEO_LINK.sub(lambda m: _player(m[2], m[1], block=False), text)
    source[0] = text


class Mermaid(SphinxDirective):
    """```mermaid blocks, kept as their source for the browser to draw."""

    has_content = True

    def run(self) -> list[nodes.Node]:
        source = html.escape("\n".join(self.content))
        return [nodes.raw("", f'<pre class="mermaid">\n{source}\n</pre>', format="html")]


# Pictures the home page shows from its own HTML (its feature wall), which Sphinx does not copy.
_HOME_PICTURES = (
    "heat-conduction.png",
    "motor-leads-emi.png",
    "x500-zurich.gif",
    "lexus-tokyo.gif",
)


def _copy_videos(app: Sphinx, error: Exception | None) -> None:
    """The players' files, at the same place under the site as under docs/, and the home page's."""
    if error is None and app.builder.format == "html":
        target = Path(app.outdir) / "videos"
        target.mkdir(parents=True, exist_ok=True)
        for video in (DOCS / "videos").glob("*.mp4"):
            shutil.copy2(video, target / video.name)
        pictures = Path(app.outdir) / "imgs"
        pictures.mkdir(parents=True, exist_ok=True)
        for name in _HOME_PICTURES:
            shutil.copy2(DOCS / "imgs" / name, pictures / name)


def setup(app: Sphinx) -> dict[str, bool]:
    app.connect("source-read", _rewrite_links)
    app.connect("build-finished", _copy_videos)
    app.add_directive("mermaid", Mermaid)
    app.add_js_file("mermaid-init.js", type="module")
    return {"parallel_read_safe": True}
