# RiskyBird documentation

Repository-owned documentation is maintained in Notion through ShyNote. When
documentation is relevant to a task, consult the configured notebook rather than
assuming an absent or ignored local Markdown file is current. Submodule docs
remain in their upstream repositories.

- Use `./tools/shynote` from this checkout. `.shynote` selects the notebook and
  `.shynote-index.json` maps repository paths to pages, folders, and figures.
  Keep those files together; never copy another branch's notebook IDs over them.
  Note and folder pages start with a `shynote-metadata` code block; leave it first
  and intact so paths can be restored reliably.
- If the runtime is missing, run `./tools/shynote --install` (requires `uv` and
  Python 3.11+). Authentication uses `NOTION_TOKEN` or `.rb/notion-token`.
  Never print or commit the token. Report missing credentials without claiming
  the remote docs were checked.
- Find relevant notes with `./tools/shynote search-title QUERY`, or look up the
  path in `.shynote-index.json`, then use `./tools/shynote read NOTE_ID`.
- For authorized edits, refresh the relevant local mapping with `pull PATH
  --diff` before editing. On a fresh checkout, use `pull PATH --id NOTE_ID` into
  an unused path. Preserve existing local edits and reconcile remote conflicts.
- Review `push PATH --dry-run --unconditional`, then publish the explicit path
  with `push PATH --unconditional`. Use the wrapper to preserve uploaded figures.
  New figures need uploading and indexing first; see `tools/shynote.md`.
- Keep notes within the appropriate directory subpages (`docs`, `workloads`,
  `hardware`, etc.). `push --all` is disabled because it would traverse submodules
  and caches. `pull --all` refreshes known mappings only, not all notes or assets.
- Agents share remote pages. Read current content before changing it, coordinate
  edits to the same note, and reconcile conflicts instead of forcing overwrites.

Keep this file, `CLAUDE.md`, `README.md`, `tools/shynote.md`, and the ShyNote configuration tracked as
bootstrap instructions. Migrated docs, `.rb/`, and `.shynote-local/` are ignored.
See `tools/shynote.md` for setup, source archives, and sync limitations.
