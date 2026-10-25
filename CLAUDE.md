# RiskyBird

@AGENTS.md

## ShyNote access

Run ShyNote only through `./tools/shynote`; that wrapper loads the private
Notion token from `.rb/notion-token` when `NOTION_TOKEN` is not exported. The
token is intentionally absent from Claude's prompt and shell environment. Do
not invoke the bare `shynote` executable or ask the user to paste the token.
Use `./tools/shynote list` to confirm Notion access (`info` only checks local
configuration). If the wrapper reports no token, report that error without
printing the token or claiming the Notion notes were checked.
