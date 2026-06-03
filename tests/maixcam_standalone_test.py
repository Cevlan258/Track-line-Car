import ast
import importlib.util
import sys
import types
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MAIN_PY = PROJECT_ROOT / "maixcam" / "main.py"


def _load_tree():
    return ast.parse(MAIN_PY.read_text(encoding="utf-8"))


def _load_main_module():
    class FakeColor:
        @staticmethod
        def from_rgb(r, g, b):
            return (r, g, b)

    maix = types.ModuleType("maix")
    maix.app = types.SimpleNamespace(need_exit=lambda: True)
    maix.camera = types.SimpleNamespace()
    maix.display = types.SimpleNamespace()
    maix.image = types.SimpleNamespace(Color=FakeColor)
    maix.pinmap = types.SimpleNamespace()
    maix.time = types.SimpleNamespace()
    maix.uart = types.SimpleNamespace()
    sys.modules["maix"] = maix

    spec = importlib.util.spec_from_file_location("maixcam_main_under_test", MAIN_PY)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _main_function(tree):
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "main":
            return node
    raise AssertionError("main() not found")


def _calls_name(node, name):
    for child in ast.walk(node):
        if isinstance(child, ast.Call):
            func = child.func
            if isinstance(func, ast.Name) and func.id == name:
                return True
            if isinstance(func, ast.Attribute) and func.attr == name:
                return True
    return False


def _class_method(tree, class_name, method_name):
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == method_name:
                    return item
    raise AssertionError(f"{class_name}.{method_name}() not found")


def _path(module, near_x, lookahead_x, far_x, rows=6, offset=0,
          lookahead_offset=0, curvature=20, branches=1, loop_score=0,
          dead_stub=False):
    return {
        "points": [{"row": index, "cx": near_x, "cy": 210 - index * 20, "w": 12}
                   for index in range(rows)],
        "rows": rows,
        "near_x": near_x,
        "near_y": 214,
        "lookahead_x": lookahead_x,
        "lookahead_y": 154,
        "far_x": far_x,
        "far_y": 74,
        "offset": offset,
        "lookahead_offset": lookahead_offset,
        "heading": far_x - near_x,
        "curvature": curvature,
        "span_x": abs(far_x - near_x),
        "branches": branches,
        "dead_stub": dead_stub,
        "loop_score": loop_score,
        "road": module.ROAD_STRAIGHT,
        "score": 0,
    }


class FakeBlob:
    def __init__(self, x, y, w, h, pixels=None):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._pixels = pixels if pixels is not None else w * h

    def x(self):
        return self._x

    def y(self):
        return self._y

    def w(self):
        return self._w

    def h(self):
        return self._h

    def pixels(self):
        return self._pixels


class FakeBlobImage:
    def __init__(self, blobs):
        self.blobs = blobs

    def find_blobs(self, *args, **kwargs):
        return list(self.blobs)


def _line(module, center_x=160):
    return {
        "valid": True,
        "offset": module.norm_offset(center_x),
        "lookahead_offset": module.norm_offset(center_x),
        "confidence": 80,
        "road": module.ROAD_STRAIGHT,
        "path": {"near_x": center_x, "lookahead_x": center_x, "far_x": center_x},
        "paths": [],
    }


def _telemetry(module, distance_mm=5200, targets=None, lora_status=None):
    return {
        "distance_mm": distance_mm,
        "lora_status": module.LORA_IDLE if lora_status is None else lora_status,
        "targets": targets or [],
    }


def _target(x_mm, y_mm, valid=1, distance_mm=None):
    if distance_mm is None:
        distance_mm = int((x_mm * x_mm + y_mm * y_mm) ** 0.5)
    return {
        "valid": valid,
        "x_mm": x_mm,
        "y_mm": y_mm,
        "speed_cm_s": 0,
        "distance_mm": distance_mm,
    }


class MaixcamStandaloneTest(unittest.TestCase):
    def test_idle_without_stm32_runs_standalone_vision_debug(self):
        source = MAIN_PY.read_text(encoding="utf-8")
        tree = _load_tree()
        main = _main_function(tree)
        idle_branches = [
            node for node in ast.walk(main)
            if isinstance(node, ast.If)
            and "telemetry is None" in ast.get_source_segment(source, node.test)
        ]

        self.assertTrue(idle_branches, "main() should have an idle/no-telemetry branch")
        self.assertTrue(any(_calls_name(branch, "run_vision_pipeline") for branch in idle_branches))

    def test_scanline_extractor_uses_dynamic_black_threshold(self):
        tree = _load_tree()
        extract = _class_method(tree, "ScanlineExtractor", "extract")

        self.assertTrue(_calls_name(extract, "build_line_threshold"))

    def test_scanline_extractor_filters_candidates_by_white_track_side(self):
        tree = _load_tree()
        extract = _class_method(tree, "ScanlineExtractor", "extract")

        self.assertTrue(_calls_name(extract, "line_has_white_side"))

    def test_debug_view_draws_scan_bands_even_without_candidates(self):
        tree = _load_tree()
        show = _class_method(tree, "SimpleDebugView", "show")

        self.assertTrue(_calls_name(show, "draw_scan_bands"))

    def test_debug_overlay_uses_maixpy_draw_rect_api(self):
        tree = _load_tree()
        called_names = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                called_names.append(node.func.attr)

        self.assertIn("draw_rect", called_names)
        self.assertNotIn("draw_rectangle", called_names)

    def test_path_graph_fallback_skips_segments_already_connected_to_path(self):
        tree = _load_tree()
        build = _class_method(tree, "PathGraph", "build")

        self.assertTrue(_calls_name(build, "path_contains_segment"))

    def test_zone_planner_uses_fixed_left_map_sign(self):
        module = _load_main_module()
        zone = module.ZonePlanner()
        zone.zone = module.ZONE_MIRROR_DISCOVERY
        right_map_path = _path(module, near_x=160, lookahead_x=215, far_x=255,
                               lookahead_offset=22)

        for _ in range(4):
            zone.update_after_path(right_map_path, 0)

        self.assertEqual(-1, zone.map_sign)

    def test_mirror_discovery_prefers_left_map_path(self):
        module = _load_main_module()
        zone = module.ZonePlanner()
        zone.zone = module.ZONE_MIRROR_DISCOVERY
        scorer = module.PathScorer()
        right_path = _path(module, near_x=158, lookahead_x=176, far_x=205,
                           offset=0, lookahead_offset=6)
        left_path = _path(module, near_x=158, lookahead_x=122, far_x=112,
                          offset=0, lookahead_offset=-16)

        selected = scorer.select([right_path, left_path], zone, 0)

        self.assertIs(selected, left_path)

    def test_circle_rect_prefers_left_exit_path(self):
        module = _load_main_module()
        zone = module.ZonePlanner()
        zone.zone = module.ZONE_CIRCLE_RECT
        scorer = module.PathScorer()
        right_path = _path(module, near_x=160, lookahead_x=178, far_x=214,
                           offset=0, lookahead_offset=7, curvature=50)
        left_path = _path(module, near_x=160, lookahead_x=124, far_x=112,
                          offset=0, lookahead_offset=-15, curvature=50)

        selected = scorer.select([right_path, left_path], zone, 0)

        self.assertIs(selected, left_path)

    def test_centered_lost_line_search_defaults_left(self):
        module = _load_main_module()
        zone = module.ZonePlanner()
        planner = module.CommandPlanner()
        line = {"valid": False, "offset": 0, "lookahead_offset": 0,
                "confidence": 0, "road": module.ROAD_UNKNOWN, "path": None,
                "paths": []}

        yaw = 0
        for _ in range(module.LOST_HOLD_FRAMES + 1):
            _, yaw, _ = planner.command_from_line(line, zone)

        self.assertLess(yaw, 0)

    def test_complex_zone_speed_can_drop_below_normal_minimum(self):
        module = _load_main_module()
        zone = module.ZonePlanner()
        zone.zone = module.ZONE_CIRCLE_RECT
        line = {
            "valid": True,
            "offset": 0,
            "confidence": 45,
            "road": module.ROAD_FORK,
            "path": {"branches": 2, "curvature": 90},
            "paths": [{}, {}, {}],
        }

        self.assertLessEqual(zone.speed_limit(line), module.LINE_COMPLEX_SPEED)

    def test_post_corridor_gate_triggers_without_crossbar(self):
        module = _load_main_module()
        detector = module.VisionGateDetector()
        img = FakeBlobImage([
            FakeBlob(82, 72, 14, 86),
            FakeBlob(224, 74, 14, 84),
        ])
        telemetry = _telemetry(module, distance_mm=5200)
        line = _line(module, center_x=160)

        events = []
        for now in (0, 80, 160):
            events.append(detector.update(img, line, telemetry, now))
        events.append(detector.update(FakeBlobImage([]), line, telemetry, 260))
        events.append(detector.update(FakeBlobImage([]), line, telemetry, 430))

        self.assertEqual([1], [event for event in events if event])
        self.assertEqual("locked", detector.state)

    def test_post_corridor_rejects_posts_that_do_not_contain_line_center(self):
        module = _load_main_module()
        detector = module.VisionGateDetector()
        img = FakeBlobImage([
            FakeBlob(24, 72, 14, 86),
            FakeBlob(116, 74, 14, 84),
        ])
        telemetry = _telemetry(module, distance_mm=5200)
        line = _line(module, center_x=200)

        for now in (0, 120, 260, 430):
            self.assertEqual(0, detector.update(img, line, telemetry, now))
        self.assertEqual("clear", detector.state)

    def test_post_corridor_rejects_horizontal_or_single_post_shapes(self):
        module = _load_main_module()
        detector = module.VisionGateDetector()
        telemetry = _telemetry(module, distance_mm=5200)
        line = _line(module, center_x=160)

        horizontal = FakeBlobImage([FakeBlob(72, 118, 160, 12)])
        single_post = FakeBlobImage([FakeBlob(88, 72, 14, 86)])

        self.assertEqual(0, detector.update(horizontal, line, telemetry, 0))
        self.assertEqual("clear", detector.state)
        self.assertEqual(0, detector.update(single_post, line, telemetry, 160))
        self.assertEqual("clear", detector.state)

    def test_post_corridor_counts_only_two_lora_gates(self):
        module = _load_main_module()
        detector = module.VisionGateDetector()
        img = FakeBlobImage([
            FakeBlob(82, 72, 14, 86),
            FakeBlob(224, 74, 14, 84),
        ])
        line = _line(module, center_x=160)
        telemetry = _telemetry(module, distance_mm=5200)

        events = []
        for base, distance in ((0, 5200), (1800, 6400), (3600, 7800)):
            telemetry["distance_mm"] = distance
            events.append(detector.update(img, line, telemetry, base))
            events.append(detector.update(img, line, telemetry, base + 160))
            events.append(detector.update(FakeBlobImage([]), line, telemetry, base + 260))
            events.append(detector.update(FakeBlobImage([]), line, telemetry, base + 430))

        self.assertEqual([1, 2], [event for event in events if event])

    def test_radar_targets_do_not_trigger_vision_gate_lora(self):
        module = _load_main_module()
        detector = module.VisionGateDetector()
        telemetry = _telemetry(module, distance_mm=5200, targets=[
            _target(-300, 700),
            _target(320, 720),
        ])
        line = _line(module, center_x=160)

        for now in (0, 160, 320, 500):
            self.assertEqual(0, detector.update(FakeBlobImage([]), line, telemetry, now))

    def test_radar_obstacle_planner_starts_from_circle_zone_without_gate_event(self):
        module = _load_main_module()
        planner = module.RadarObstaclePlanner()
        zone = module.ZonePlanner()
        zone.zone = module.ZONE_CIRCLE_RECT
        telemetry = _telemetry(module, distance_mm=8500, targets=[
            _target(-360, 760),
            _target(-420, 900),
            _target(-390, 1040),
        ])

        for now in (0, 50, 100):
            planner.update(telemetry, zone, now)

        self.assertTrue(planner.active)
        self.assertEqual(module.OBSTACLE_SIDE_LEFT, planner.obstacle_side)
        self.assertEqual(module.AVOID_SIDE_RIGHT, planner.avoid_side)


if __name__ == "__main__":
    unittest.main()
