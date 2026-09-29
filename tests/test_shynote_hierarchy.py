"""Run with .rb/shynote/bin/python -m unittest tests.test_shynote_hierarchy."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

try:
    from shynote.config import NotionConfig
    from shynote.model import NotFound, ShyNoteError
    from tools.shynote_hierarchy import HierarchicalNotionStore
except ImportError as exc:
    raise unittest.SkipTest("Install the optional ShyNote environment first") from exc


ROOT = "00000000-0000-4000-8000-000000000001"
FOLDER = "00000000-0000-4000-8000-000000000002"
NOTE = "00000000-0000-4000-8000-000000000003"
OTHER = "00000000-0000-4000-8000-000000000004"
ASSETS = "00000000-0000-4000-8000-000000000005"


def page(note_id, parent):
    return {"id": note_id, "parent": {"type": "page_id", "page_id": parent}}


def child(note_id, title):
    return {"id": note_id, "type": "child_page", "child_page": {"title": title}}


class Transport:
    def __init__(self):
        self.pages = {FOLDER: page(FOLDER, ROOT), NOTE: page(NOTE, FOLDER),
                      OTHER: {"id": OTHER, "parent": {"type": "workspace"}}}
        self.children = {ROOT: [child(FOLDER, "docs"), child(ASSETS, "Figures")],
                         FOLDER: [child(NOTE, "setup.md")]}

    def request(self, method, path, payload=None):
        assert method == "GET", "These operations must remain read-only"
        if path.startswith("pages/"):
            return self.pages[path.split("/")[1]]
        return {"results": self.children[path.split("/")[1]], "has_more": False}


class HierarchyTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        (root / ".shynote-index.json").write_text(json.dumps({
            "parent_page_id": ROOT, "folders": {"docs": FOLDER}, "asset_pages": [ASSETS]}))
        self.transport = Transport()
        config = SimpleNamespace(root=root, storage=NotionConfig(ROOT, "NOTION_TOKEN"))
        self.store = HierarchicalNotionStore(config, transport=self.transport)

    def test_nested_note_is_accessible(self):
        self.assertEqual(self.store._page(NOTE)["id"], NOTE)

    def test_note_moved_to_another_branch_is_rejected(self):
        self.transport.pages[FOLDER] = page(FOLDER, OTHER)
        with self.assertRaises(NotFound):
            self.store._page(NOTE)

    def test_archived_folder_is_rejected(self):
        self.transport.pages[FOLDER]["archived"] = True
        with self.assertRaises(NotFound):
            self.store._page(NOTE)

    def test_list_preserves_paths_and_excludes_asset_gallery(self):
        notes = self.store.list_notes()
        self.assertEqual([(n.id, n.title) for n in notes], [(NOTE, "docs/setup.md")])

    def test_search_finds_notes_inside_folders(self):
        self.assertEqual([n.id for n in self.store.search_title("SETUP")], [NOTE])

    def media_index(self):
        self.store.index["documents"] = {"docs/setup.md": NOTE}
        self.store.index["assets"] = {
            "docs/figures/chart.svg": {"filename": "docs__figures__chart.svg", "upload_id": ASSETS}}

    def test_expiring_image_urls_do_not_create_false_changes(self):
        self.media_index()
        prefix = "![Chart](https://prod-files-secure.s3.us-west-2.amazonaws.com/files/docs__figures__chart.svg?signature="
        old = self.store._image_body(prefix + "old)", NOTE)
        new = self.store._image_body(prefix + "new)", NOTE)
        self.assertEqual(old, "![Chart](figures/chart.svg)")
        self.assertEqual(new, old)

    def test_existing_local_figure_is_prepared_for_native_attachment(self):
        self.media_index()
        result = self.store._image_body("![Chart](figures/chart.svg)", NOTE, outgoing=True)
        self.assertIn("https://riskybird.invalid/asset/docs/figures/chart.svg", result)

    def test_link_in_image_caption_does_not_hide_expiring_url(self):
        self.media_index()
        caption = "Rendered with [frame-to-colour.py](http://frame-to-colour.py/)"
        prefix = "![" + caption + "](https://prod-files-secure.s3.us-west-2.amazonaws.com/files/docs__figures__chart.svg?signature="
        expected = "![" + caption + "](figures/chart.svg)"
        self.assertEqual(self.store._image_body(prefix + "old)", NOTE), expected)
        self.assertEqual(self.store._image_body(prefix + "new)", NOTE), expected)
        prepared = self.store._image_body(expected, NOTE, outgoing=True)
        self.assertIn("https://riskybird.invalid/asset/docs/figures/chart.svg", prepared)

    def test_new_local_figure_is_not_silently_published_as_broken_link(self):
        self.media_index()
        with self.assertRaises(ShyNoteError):
            self.store._image_body("![New](figures/new.png)", NOTE, outgoing=True)

    def test_image_syntax_in_code_example_stays_literal(self):
        self.media_index()
        body = "```markdown\n![New](figures/new.png)\n```\n"
        self.assertEqual(self.store._image_body(body, NOTE, outgoing=True), body)

    def test_document_heading_link_survives_a_local_update(self):
        self.media_index()
        self.store.index["headings"] = {"docs/setup.md": {"install": "heading-block"}}
        body = self.store._image_body("[Install](#install)", NOTE, outgoing=True)
        self.assertEqual(body, "[Install](https://www.notion.so/" + NOTE.replace("-", "") + "#heading-block)")

    def test_quoted_command_is_sent_as_literal_code(self):
        self.media_index()
        body = '> ```console\n> echo "$VALUE" | cat\n> ```\n'
        prepared = self.store._image_body(body, NOTE, outgoing=True)
        self.assertIn('```bash\necho "$VALUE" | cat\n```', prepared)


if __name__ == "__main__":
    unittest.main()
