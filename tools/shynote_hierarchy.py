"""Keep ShyNote scoped to one branch and preserve repository asset links.

Upstream stores paths in page metadata and supports nested Notion directories.
This adapter normalizes indexed figure links and excludes the separate asset
gallery from the repository note tree.
"""
import json
from dataclasses import replace
import posixpath
import re
import time
from urllib.parse import unquote, urlsplit

from markdown_it import MarkdownIt
from shynote.model import NoteSummary, NotFound, ProviderError, ShyNoteError
from shynote.stores.notion import NotionStore, NotionTransport


class PacedTransport(NotionTransport):
    def __init__(self, token_env):
        super().__init__(token_env)
        self.last_request = 0.0

    def request(self, method, path, payload=None):
        for attempt in range(5):
            time.sleep(max(0, 0.36 - (time.monotonic() - self.last_request)))
            self.last_request = time.monotonic()
            try:
                return super().request(method, path, payload)
            except ProviderError as exc:
                if "HTTP 429" not in str(exc) or attempt == 4:
                    raise
                delay = re.search(r"Retry after (\d+) seconds", str(exc))
                time.sleep(min(60, int(delay[1]) + 1 if delay else 3))


class HierarchicalNotionStore(NotionStore):
    def __init__(self, config, *, transport=None):
        super().__init__(config, transport=transport or PacedTransport(config.storage.token_env))
        self.root = config.root
        self.index = {}
        path = config.root / ".shynote-index.json"
        if path.exists():
            index = json.loads(path.read_text())
            if index["parent_page_id"] != self.settings.parent_page_id:
                raise ShyNoteError("The document index belongs to a different Notion notebook.")
            self.index = index

    def _image_body(self, body, note_id, outgoing=False):
        """Use repository paths locally, permanent uploaded files in Notion.

        Signed image URLs change independently of page edits. Converting known
        assets back to local paths also prevents false ShyNote conflicts.
        """
        paths = {self._id(value): key for key, value in self.index.get("documents", {}).items()}
        name = paths.get(self._id(note_id))
        if name is None:
            return body
        if outgoing:
            lines = body.splitlines(keepends=True)
            for token in reversed(MarkdownIt().parse(body)):
                if token.type == "fence" and token.map and lines[token.map[0]].lstrip().startswith(">"):
                    start, end = token.map
                    language = "bash" if token.info in {"sh", "shell", "console"} else token.info
                    lines[start:end] = ["\n```" + language + "\n", token.content, "```\n\n"]
            body = "".join(lines)
        assets = self.index.get("assets", {})
        filenames = {value["filename"]: key for key, value in assets.items()}

        def image(match):
            caption, target = match.groups()
            parsed = urlsplit(target)
            if outgoing and not parsed.scheme and not parsed.netloc:
                key = posixpath.normpath(posixpath.join(posixpath.dirname(name), unquote(parsed.path)))
                if key not in assets:
                    raise ShyNoteError(f"Upload the new figure before pushing this document: {key}")
                return f"![{caption}](https://riskybird.invalid/asset/{key})"
            if not outgoing and parsed.hostname and parsed.hostname.startswith("prod-files-secure."):
                key = filenames.get(unquote(parsed.path.rsplit("/", 1)[-1]))
                if key:
                    relative = posixpath.relpath(key, posixpath.dirname(name) or ".")
                    return f"![{caption}]({relative})"
            return match.group(0)

        def link(match):
            label, target = match.groups()
            parsed = urlsplit(target)
            if parsed.scheme or parsed.netloc:
                return match.group(0)
            key = posixpath.normpath(posixpath.join(posixpath.dirname(name), unquote(parsed.path))) if parsed.path else name
            document = self.index.get("documents", {}).get(key)
            if document:
                address = "https://www.notion.so/" + document.replace("-", "")
                anchor = self.index.get("headings", {}).get(key, {}).get(unquote(parsed.fragment))
                if anchor:
                    address += "#" + anchor
                return f"[{label}]({address})"
            if key in assets and self.index.get("asset_pages"):
                address = "https://www.notion.so/" + self.index["asset_pages"][0].replace("-", "")
                return f"[{label}]({address}#{assets[key]['block_id'].replace('-', '')})"
            return match.group(0)

        # Code examples must remain literal.
        chunks = re.split(r"(^[ \t]*```[^\n]*\n.*?^[ \t]*```[^\n]*(?:\n|$))", body, flags=re.M | re.S)
        for i in range(0, len(chunks), 2):
            # Notion can auto-link text inside an image caption. Allow the
            # caption's nested brackets so the outer image URL is normalized.
            chunks[i] = re.sub(r"!\[((?:[^\[\]\n]|\[[^\[\]\n]*\])*)\]\(([^)\n]+)\)", image, chunks[i])
            if outgoing:
                chunks[i] = re.sub(r"(?<!!)\[([^\]\n]*)\]\(([^)\n]+)\)", link, chunks[i])
        return "".join(chunks)

    def read(self, note_id):
        note = super().read(note_id)
        return replace(note, body=self._image_body(note.body, note.id))

    def update(self, note_id, body, *, revision=None, unconditional=False):
        prepared = self._image_body(body, note_id, outgoing=True)
        acknowledged = super().update(note_id, prepared, revision=revision, unconditional=unconditional)
        if "https://riskybird.invalid/asset/" not in prepared:
            return self._image_body(acknowledged, note_id)

        def attach(parent):
            for block in self._children(parent):
                kind = block["type"]
                if kind == "image" and block[kind]["type"] == "external":
                    target = block[kind]["external"]["url"]
                    if target.startswith("https://riskybird.invalid/asset/"):
                        key = unquote(target.split("/asset/", 1)[1])
                        media = self.index["assets"][key]
                        self.transport.request("PATCH", f"blocks/{block['id']}", {
                            "image": {"file_upload": {"id": media["upload_id"]},
                                      "caption": block[kind].get("caption", [])}})
                if block.get("has_children") and kind not in {"child_page", "child_database"}:
                    attach(block["id"])

        attach(self._id(note_id))
        result = self.read(note_id).body
        if "https://riskybird.invalid/asset/" in result:
            raise ProviderError("An image attachment is incomplete; inspect before retrying.")
        return result

    def _page(self, note_id):
        note_id = self._id(note_id)
        page = self.transport.request("GET", f"pages/{note_id}")
        ancestor = page
        seen = {note_id}
        while True:
            parent = ancestor.get("parent", {})
            if parent.get("type") != "page_id":
                raise NotFound("Note is outside this notebook.")
            parent_id = self._id(parent["page_id"])
            if parent_id == self.settings.parent_page_id:
                return page
            if parent_id in seen or len(seen) >= 64:
                raise ProviderError("Invalid Notion page ancestry.")
            seen.add(parent_id)
            ancestor = self.transport.request("GET", f"pages/{parent_id}")
            if self._archived(ancestor):
                raise NotFound("A parent folder is archived.")

    def _children(self, parent):
        cursor = None
        seen = set()
        while True:
            from urllib.parse import urlencode

            query = {"page_size": 100}
            if cursor:
                query["start_cursor"] = cursor
            result = self.transport.request("GET", f"blocks/{parent}/children?{urlencode(query)}")
            yield from result["results"]
            if not result.get("has_more"):
                return
            cursor = result.get("next_cursor")
            if not cursor or cursor in seen:
                raise ProviderError("Invalid Notion pagination cursor.")
            seen.add(cursor)

    def list_notes(self):
        excluded = set(self.index.get("asset_pages", []))
        notes = []
        visited = set()
        paths = set()

        def visit(parent, parent_path=""):
            if parent in visited:
                raise ProviderError("Repeated Notion page in notebook hierarchy.")
            visited.add(parent)
            for block in self._children(parent):
                if block["type"] != "child_page" or self._archived(block):
                    continue
                note_id = self._id(block["id"])
                if note_id in excluded:
                    continue
                title = block["child_page"]["title"]
                metadata = self._metadata(note_id)
                self._check_location(note_id, title, metadata, parent_path)
                if metadata["kind"] == "directory":
                    visit(note_id, metadata["path"])
                else:
                    if metadata["path"] in paths:
                        raise ProviderError(f"Duplicate Notion note path {metadata['path']!r}.")
                    paths.add(metadata["path"])
                    notes.append(NoteSummary(note_id, title, metadata["path"]))

        visit(self.settings.parent_page_id)
        return notes

    def search_title(self, query):
        if not isinstance(query, str) or not query.strip():
            raise ShyNoteError("Title search requires a nonempty query.")
        needle = query.strip().casefold()
        return [note for note in self.list_notes() if needle in note.title.casefold()]


def main():
    # open_notebook imports this class at call time; keep the pinned package intact.
    import shynote.stores.notion
    from shynote.cli import main as cli

    shynote.stores.notion.NotionStore = HierarchicalNotionStore
    return cli()


if __name__ == "__main__":
    raise SystemExit(main())
