# RiskyBird documentation

Repository-owned documentation is maintained in Notion through ShyNote. When
documentation is relevant to a task, consult the configured notebook rather than
assuming an absent or ignored local Markdown file is current. Submodule docs
remain in their upstream repositories.

- Use `./tools/shynote` from this checkout. `.shynote` selects the notebook and
  `tools/shynote-index.json` maps repository paths to pages, folders, and figures.
  Keep those files together; never copy another branch's notebook IDs over them.
  Note and folder pages start with a `shynote-metadata` code block; leave it first
  and intact so paths can be restored reliably.
- Run ShyNote only through `./tools/shynote`; that wrapper loads the private
  Notion token from `.rb/notion-token` when `NOTION_TOKEN` is not exported. The
  token is intentionally absent from the agent's prompt and shell environment. Do
  not invoke the bare `shynote` executable or ask the user to paste the token.
  Use `./tools/shynote list` to confirm Notion access (`info` only checks local
  configuration).
- If the runtime is missing, run `./tools/shynote --install` (requires `uv` and
  Python 3.11+). Authentication uses `NOTION_TOKEN` or `.rb/notion-token`.
  Never print or commit the token. If the wrapper reports no token, report that
  error without printing the token or claiming the Notion notes were checked.
- Find relevant notes with `./tools/shynote search-title QUERY`, or look up the
  path in `tools/shynote-index.json`, then use `./tools/shynote read NOTE_ID`.
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

## Developing the backends

Full guide: [`backends/README.md`](backends/README.md). Backends are pinned at plain tips. riskybird-dev wraps them locally and commits nothing inside them.
- RiskyBird code goes in `hardware/`, `workloads/` and `tools/`, never in `backends/`. The wrappers:
  `backends/overlays` (symlinks into backends), the sbt plugin `hardware/chipyard/project/RiskyBird.scala`,
  `hardware/chipyard/fpga/rb-fpga.mk`, the patch queue `hardware/chipyard/patches/` (generic Chipyard-tree
  fixes only), `backends/redirects` (nested duplicates become links), and rb's CMake arguments
  (`hardware/zephyr` overlays, Kconfig and bindings; `workloads/lib` via `RB_ROOT`).
- The queue reaches **only** `backends/chipyard` and its nested repos. A change to Zephyr, zephyr-chipyard-sw,
  ModelBlaster or RoSE source is a commit on that repo's `rb-clean` (never `dev`), followed by a separate
  riskybird-dev PR that only bumps the pin. Zephyr is two levels: zephyr `rb-clean`, then a zcs `rb-clean`
  bump, then riskybird-dev.
- Work in a standalone clone. Linked worktrees share `.git/config`, so never run `tools/backend-patches apply`
  or `git submodule sync` from one. Init submodules per tier, never `--recursive`. The bench tier needs
  `backends/zephyr-chipyard-sw` **and** `backends/modelblaster`.
- Setup: `source ./rb-env.sh`, then `rb env setup zephyr` and `rb env setup chipyard`. After any submodule
  move, run `tools/submodule-redirects apply` and then `tools/backend-patches apply`.
- Before a PR:
  - `tools/submodule-redirects check` and `tools/backend-patches check` exit 0.
  - `PYTHONPATH=. python3 -m unittest discover -s tests -t tests` passes.
  - Behaviour-neutral changes produce byte-identical `objcopy -O binary` images (objcopy from the build's
    `CMAKE_OBJCOPY`) for `rb build flight_controller --target arty200t --mode esp-motors --config
    RocketArty200TDroneFullDDRLoaderConfig`.
  - `git diff --cached --submodule` shows only the gitlinks you meant to move.
- Never use `git add -A` or `commit -a` (they stage pin moves), trust `git submodule status` for redirected paths,
  or run Chipyard's full `build-setup.sh`. Changes reach `main` only by PR, and only when the user asks.

Keep this file, `CLAUDE.md`, `README.md`, `backends/README.md`, `tools/shynote.md`, and the ShyNote configuration tracked as
bootstrap instructions. Migrated docs, `.rb/`, and `.shynote-local/` are ignored.
See `tools/shynote.md` for setup, source archives, and sync limitations.
