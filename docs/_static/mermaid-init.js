// Draws the site's ```mermaid blocks (docs/conf.py writes them as <pre class="mermaid">), one at
// a time, each with an id of its own (Mermaid draws by id, and two diagrams with the same id
// end up drawn into one). Draws them again when the
// reader switches between light and dark.
import mermaid from "https://cdn.jsdelivr.net/npm/mermaid@11.12.1/dist/mermaid.esm.min.mjs";

const blocks = [...document.querySelectorAll("pre.mermaid")];
for (const block of blocks) {
  block.dataset.source = block.textContent;
}

const dark = () => {
  const theme = document.body.dataset.theme;
  return theme === "dark" ||
    (theme !== "light" && window.matchMedia("(prefers-color-scheme: dark)").matches);
};

let queue = Promise.resolve();
let pass = 0;
const draw = () => {
  queue = queue.then(async () => {
    mermaid.initialize({ startOnLoad: false, theme: dark() ? "dark" : "default" });
    pass += 1;
    for (const [index, block] of blocks.entries()) {
      // An id of its own per diagram and pass: Mermaid draws into the element with that id.
      const id = `lagrange-mermaid-${pass}-${index}`;
      try {
        const { svg, bindFunctions } = await mermaid.render(id, block.dataset.source);
        block.innerHTML = svg;
        bindFunctions?.(block);
      } catch (error) {
        console.error("mermaid:", error);
        block.textContent = block.dataset.source;
      }
    }
  });
};

draw();
new MutationObserver(draw).observe(document.body, { attributes: true, attributeFilter: ["data-theme"] });
window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", draw);
