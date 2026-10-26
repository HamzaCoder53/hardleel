"""robot_slam.carto_lua: macOS workaround that inlines Cartographer `include "x.lua"` lines."""
import os
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from robot_slam import carto_lua  # noqa: E402


class CartoLuaTest(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp()
        w = lambda n, t: open(os.path.join(self.d, n), 'w').write(t)
        w('pose_graph.lua', 'POSE_GRAPH = {a = 1}\nreturn POSE_GRAPH\n')
        w('map_builder.lua', 'include "pose_graph.lua"\nMAP_BUILDER = {pg = POSE_GRAPH}\nreturn MAP_BUILDER\n')
        w('cfg.lua', 'include "map_builder.lua"\ninclude "pose_graph.lua"\noptions = {mb = MAP_BUILDER}\nreturn options\n')

    def test_includes_inlined_once_and_included_returns_dropped(self):
        text = carto_lua._inline('cfg.lua', [self.d], {'cfg.lua'}, root=True)
        self.assertNotIn('include', text)
        self.assertEqual(text.count('POSE_GRAPH = {a = 1}'), 1)          # pose_graph.lua pulled in twice, inlined once
        self.assertLess(text.index('POSE_GRAPH ='), text.index('MAP_BUILDER ='))
        self.assertEqual(text.count('return'), 1)                         # only the root's `return options` survives
        self.assertTrue(text.rstrip().endswith('return options'))

    def test_unchanged_on_linux(self):
        with mock.patch.object(sys, 'platform', 'linux'):
            self.assertEqual(carto_lua.config_for_platform(self.d, 'cfg.lua'), (self.d, 'cfg.lua'))

    def test_flattened_copy_on_macos(self):
        with mock.patch.object(sys, 'platform', 'darwin'):
            d, b = carto_lua.config_for_platform(self.d, 'cfg.lua')
        self.assertNotEqual(d, self.d)
        self.assertNotIn('include', open(os.path.join(d, b)).read())

    def test_missing_include_raises(self):
        open(os.path.join(self.d, 'bad.lua'), 'w').write('include "nope.lua"\nreturn {}\n')
        with self.assertRaises(FileNotFoundError):
            carto_lua._inline('bad.lua', [self.d], {'bad.lua'}, root=True)


if __name__ == '__main__':
    unittest.main()
