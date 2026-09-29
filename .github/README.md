# RiskyBird

Repository-owned documentation lives in the
[Riskybird Docs notebook](https://www.notion.so/3ea06b97ede480799eb4d570d4d4d4be).
Open the [integration branch documentation](https://www.notion.so/3ea06b97ede4817fb997d1c5f607c577)
for setup, flight, hardware, workloads, experiments, and research guides.
Submodule documentation remains with its upstream repository.

Each branch has its own notebook section. Within a section, subpages follow the
repository directories, including `docs/history`, `workloads`, `hardware`, and
`dse`. The **Figures and source files** page contains hosted figures, supporting
data, and a downloadable ZIP of the original documents and assets.

## Access from this checkout

Install the pinned ShyNote CLI with Python 3.11+ and `uv`:

```sh
./tools/shynote --install
./tools/shynote info
./tools/shynote search-title setup
./tools/shynote read NOTE_ID
```

Supply `NOTION_TOKEN` in the environment, or save its raw value in the ignored
`.rb/notion-token` file with permissions `0600`. The Notion connection must have
access to the notebook's parent page. `.shynote` selects this branch's notebook;
`.shynote-index.json` records its document, folder, and asset mappings.

File arguments are relative to the repository root, for example
`./tools/shynote pull docs/setup.md --diff`. New checkouts should use `read`, or
pull into an unused path with `--id NOTE_ID`. `pull --all` only refreshes existing
local mappings; it does not download the notebook or its attachments.

Edit pages in Notion, or review local Markdown changes before publishing:

```sh
./tools/shynote push docs/setup.md --dry-run --unconditional
./tools/shynote push docs/setup.md --unconditional
```

The local adapter finds nested notes and preserves already-uploaded figures.
New local figures must be uploaded and added to the asset index first. Broad
`push --all` is disabled because it would include backend submodules and caches.
Notion does not support atomic conditional writes; review conflicts rather than
overwriting concurrent edits. HTML and reStructuredText were converted for
reading in Notion; the original formats are retained in the source ZIP.

Local docs and figures can remain on disk after migration; Git ignores these
working copies. Do not commit `.rb/` or `.shynote-local/`.

## Agents and other checkouts

`AGENTS.md` tells Codex and compatible agents how to find and update this checkout's
Notion docs. `CLAUDE.md` imports the same file for Claude Code. Start a new agent
session after receiving these instructions, or have an existing agent read
`AGENTS.md`. In Claude Code, `/context` shows loaded memory files. Other agent
tools may need to be told to read `AGENTS.md`.

The four existing worktrees on this machine share the installed ShyNote runtime
and token. A fresh clone or worktree needs `./tools/shynote --install` and access
to `NOTION_TOKEN` or its own ignored `.rb/notion-token`. No per-agent installation
is needed when agents share a configured checkout. Separate machines or isolated
agent environments need their own setup and network access to Notion.

The config travels with each branch once its commit is available in that checkout.
ShyNote does not automatically synchronize edits between branches. A new branch
inherits its starting branch's notebook until deliberately configured otherwise.
