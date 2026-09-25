"""Regression tests for the XPU-RT schedule-and-plot adapter.

The scheduler itself needs numpy and a populated profile tree, so these tests
pin the parts that are ours: discovery across the two profile layouts, spec
generation, the symlinked mirror, and the schedule-JSON parser.
"""

from __future__ import annotations

import csv
import json
import os
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tools.rb import xpurt
from tools.rb.boards import ROOT

# The XPU-RT backend and the dse/ tree live on main. The setup branch has only
# the zephyr-chipyard-sw profiles, so tests that need either skip there.
needs_xpurt = unittest.skipUnless(
    (xpurt.XPURT / "scripts" / "run_xpurt_schedule.py").is_file(),
    "backends/XPU-RT lives on main; not on the setup branch",
)
needs_dse_workloads = unittest.skipUnless(
    xpurt.WORKLOAD_DIR.is_dir(),
    "dse/workloads lives on main; not on the setup branch",
)
needs_profile_targets = unittest.skipUnless(
    xpurt.PROFILE_TARGET_DIR.is_dir(),
    "dse/profile-targets lives on main; not on the setup branch",
)
needs_forest_schedule = unittest.skipUnless(
    xpurt.FOREST_SCHEDULE.is_file(),
    "the forest schedule is in backends/XPU-RT, which lives on main; not on the setup branch",
)


class DiscoveryTests(unittest.TestCase):
    @needs_xpurt
    def test_profiles_are_found_across_both_layouts(self) -> None:
        """Chipyard sweeps sit under a sweep tag; the QNN tree does not."""
        profiles = xpurt.discover_profiles()
        self.assertTrue(profiles, "no profiles discovered at all")
        keys = {(p.hw, p.target, p.model) for p in profiles}
        # Both backend kinds, all three flight networks, on the FireSim target.
        for hw in ("gemmini_q31", "V256D128_rvv"):
            for model in ("dronet", "yolov8_nano", "mlp_control"):
                with self.subTest(hw=hw, model=model):
                    self.assertIn((hw, "firesim_rocket_saturn", model), keys)

    def test_every_discovered_profile_points_at_a_real_results_csv(self) -> None:
        for prof in xpurt.discover_profiles():
            with self.subTest(key=prof.key):
                self.assertTrue(prof.results.is_file())
                self.assertEqual(prof.results.name, "results.csv")

    def test_profiles_for_filters_on_target_and_hw(self) -> None:
        got = xpurt.profiles_for("firesim_rocket_saturn", "gemmini_q31")
        self.assertEqual(
            set(got), {"dronet", "yolov8_nano", "mlp_control"}
        )

    def test_discovery_does_not_invent_targets(self) -> None:
        """Scanning one level down for a sweep tag must not re-parse the
        untagged tree at the wrong depth, which turned model names like
        `mlp_control` into profiling targets."""
        targets = {p.target for p in xpurt.discover_profiles()}
        for model in ("dronet", "yolov8_nano", "mlp_control"):
            with self.subTest(model=model):
                self.assertNotIn(model, targets)
        self.assertIn("firesim_rocket_saturn", targets)

    def test_dispatch_graphs_are_found(self) -> None:
        graphs = {(g.model, g.target) for g in xpurt.discover_graphs()}
        self.assertIn(("dronet", "firesim_rocket_saturn"), graphs)
        self.assertIn(("yolov8_nano", "firesim_rocket_saturn"), graphs)
        self.assertIn(("mlp_control", "generic_riscv64"), graphs)


class WorkloadTests(unittest.TestCase):
    @needs_dse_workloads
    def test_flight_workloads_resolve_to_real_graphs(self) -> None:
        for wl in xpurt.load_workloads(["mlp_control", "dronet", "yolov8_nano"]):
            with self.subTest(workload=wl.name):
                self.assertTrue(wl.graph.is_file(), f"{wl.graph} missing")

    @needs_dse_workloads
    def test_mpc_is_the_only_safety_critical_entry(self) -> None:
        loaded = xpurt.load_workloads(["mlp_control", "dronet", "yolov8_nano"])
        critical = [w.name for w in loaded if w.criticality == "safety-critical"]
        self.assertEqual(critical, ["mlp_control"])

    @needs_dse_workloads
    def test_periodic_workloads_carry_a_deadline(self) -> None:
        for wl in xpurt.load_workloads(["mlp_control", "dronet"]):
            with self.subTest(workload=wl.name):
                self.assertIsNotNone(wl.period)
                self.assertIsNotNone(wl.window_duration)

    def test_unknown_workload_lists_what_exists(self) -> None:
        with self.assertRaises(xpurt.XpurtError) as caught:
            xpurt.load_workloads(["nonexistent"])
        self.assertIn("available", str(caught.exception))


class SpecTests(unittest.TestCase):
    @needs_dse_workloads
    def test_dispatch_paths_are_absolute(self) -> None:
        """Relative paths resolve against XPU-RT's base, which lacks our layout."""
        with TemporaryDirectory() as tmp:
            ws = Path(tmp)
            (ws / "data" / "toplevel").mkdir(parents=True)
            loaded = xpurt.load_workloads(["dronet", "mlp_control"])
            spec = xpurt.write_spec(
                ws, "t", loaded,
                target="firesim_rocket_saturn",
                hw_p="gemmini_q31", hw_e="V256D128_rvv",
            )
            data = json.loads(spec.read_text())
        for name, net in data["networks"].items():
            with self.subTest(network=name):
                path = net["dispatch_deps_path"]
                self.assertTrue(os.path.isabs(path), f"{path} is not absolute")
                self.assertTrue(Path(path).is_file())

    @needs_dse_workloads
    def test_spec_records_hardware_pairing_and_periods(self) -> None:
        with TemporaryDirectory() as tmp:
            ws = Path(tmp)
            (ws / "data" / "toplevel").mkdir(parents=True)
            spec = xpurt.write_spec(
                ws, "t", xpurt.load_workloads(["mlp_control", "yolov8_nano"]),
                target="firesim_rocket_saturn",
                hw_p="gemmini_q31", hw_e="V256D128_rvv",
            )
            data = json.loads(spec.read_text())
        self.assertEqual(data["hardware"]["profile_hw"]["cpu_p"], "gemmini_q31")
        self.assertEqual(data["hardware"]["profile_hw"]["cpu_e"], "V256D128_rvv")
        self.assertTrue(data["scheduler"]["use_profiled"])
        # mlp_control is periodic, yolov8_nano is one-shot.
        self.assertEqual(data["networks"]["mlp_control"]["period"], 20)
        self.assertNotIn("period", data["networks"]["yolov8_nano"])

    @needs_dse_workloads
    def test_derived_machine_block_replaces_the_hand_written_one(self) -> None:
        """A single-hart SoC gets one core and no CPU_E, not a typed-in pair."""
        with TemporaryDirectory() as tmp:
            ws = Path(tmp)
            (ws / "data" / "toplevel").mkdir(parents=True)
            spec = xpurt.write_spec(
                ws, "t", xpurt.load_workloads(["mlp_control"]),
                target="firesim_rocket_saturn",
                machines={"CPU_P": 1},
                profile_hw={"cpu_p": "novec-gemmini1024-abc123"},
            )
            data = json.loads(spec.read_text())
        self.assertEqual(data["hardware"]["machines"], {"cpu_p": 1})
        self.assertEqual(
            data["hardware"]["profile_hw"], {"cpu_p": "novec-gemmini1024-abc123"}
        )

    def test_a_soc_with_no_cores_is_refused(self) -> None:
        with TemporaryDirectory() as tmp:
            ws = Path(tmp)
            (ws / "data" / "toplevel").mkdir(parents=True)
            with self.assertRaises(xpurt.XpurtError):
                xpurt.write_spec(
                    ws, "t", xpurt.load_workloads(["mlp_control"]),
                    target="firesim_rocket_saturn", machines={}, profile_hw={},
                )

    def test_topo_tag_follows_the_core_count(self) -> None:
        """Reading topo_0 for a two-core machine reports single-core timings."""
        self.assertEqual(xpurt.topo_tag_for_cores(1), "topo_0")
        self.assertEqual(xpurt.topo_tag_for_cores(2), "topo_0_1")
        self.assertEqual(xpurt.topo_tag_for_cores(4), "topo_0_1_2_3")

    @needs_dse_workloads
    def test_spec_topo_tag_is_derived_from_the_machine(self) -> None:
        with TemporaryDirectory() as tmp:
            ws = Path(tmp)
            (ws / "data" / "toplevel").mkdir(parents=True)
            spec = xpurt.write_spec(
                ws, "t", xpurt.load_workloads(["mlp_control"]),
                target="firesim_rocket_saturn",
                machines={"CPU_P": 2}, profile_hw={"cpu_p": "x"},
            )
            data = json.loads(spec.read_text())
        self.assertEqual(data["hardware"]["profile"]["topo_tag"], "topo_0_1")

    def test_the_flight_profiles_cover_only_one_core(self) -> None:
        """So a multi-core config cannot be scheduled against them yet."""
        for prof in xpurt.discover_profiles():
            if prof.target != "firesim_rocket_saturn":
                continue
            with self.subTest(key=prof.key):
                self.assertEqual(xpurt.available_topos(prof), {"topo_0"})

    def test_provenance_records_which_profile_a_number_came_from(self) -> None:
        """Two families of the same network disagree by ~4.6x in this tree."""
        prof = [
            p for p in xpurt.discover_profiles()
            if p.hw == "gemmini_q31" and p.model == "dronet"
        ][0]
        got = xpurt.profile_provenance(prof)
        self.assertTrue(got["has_cycles"])
        self.assertEqual(got["rows"], 30)
        self.assertEqual(got["backend_root"], "zephyr-chipyard-sw")
        self.assertEqual(got["sweep_tag"], "sweep_v8")
        self.assertIn("conv2d_s8", got["ops"])
        self.assertIn("gen/profile", got["path"])

    def test_workload_ops_come_from_the_profile_not_the_graph(self) -> None:
        """Dispatch graphs carry only ids and dependencies."""
        ops = xpurt.workload_ops("firesim_rocket_saturn", "dronet")
        self.assertIn("conv2d_s8", ops)
        self.assertIn("maxpool2d_s8", ops)
        self.assertIn("sigmoid_s8", ops)

    def test_missing_graph_is_refused(self) -> None:
        with TemporaryDirectory() as tmp:
            ws = Path(tmp)
            (ws / "data" / "toplevel").mkdir(parents=True)
            bogus = xpurt.Workload(name="ghost", graph=Path(tmp) / "nope.json")
            with self.assertRaises(xpurt.XpurtError):
                xpurt.write_spec(
                    ws, "t", [bogus], target="t", hw_p="a", hw_e="b"
                )


class WorkspaceTests(unittest.TestCase):
    """Built against a throwaway mirror.

    `link_profiles` deletes retimed output it finds in its way, so pointing
    these at the live `dse/results/xpurt` would let a test run clobber the
    inputs of a scheduler running alongside it.
    """

    def setUp(self) -> None:
        self._tmp = TemporaryDirectory()
        self._real = xpurt.WORKSPACE
        xpurt.WORKSPACE = Path(self._tmp.name) / "xpurt"

    def tearDown(self) -> None:
        xpurt.WORKSPACE = self._real
        self._tmp.cleanup()

    @needs_xpurt
    def test_mirror_symlinks_backend_code_and_owns_the_rest(self) -> None:
        """abspath does not follow symlinks, so the mirror becomes the repo base."""
        ws = xpurt.build_workspace()
        for name in ("scripts", "xpu-rt"):
            with self.subTest(entry=name):
                self.assertTrue((ws / name).is_symlink())
                self.assertEqual(
                    Path(os.readlink(ws / name)), xpurt.XPURT / name
                )
        for name in ("gen/profile", "data/toplevel", "schedules", "plots"):
            with self.subTest(entry=name):
                self.assertTrue((ws / name).is_dir())
                self.assertFalse((ws / name).is_symlink())

    def test_mirror_leaves_the_backend_untouched(self) -> None:
        """XPU-RT's gen/ tree is tracked; the flow must not write into it."""
        # The configured location, not the temporary one this case runs against.
        self.assertFalse(str(self._real).startswith(str(xpurt.XPURT)))
        self.assertTrue(str(self._real).startswith(str(ROOT / "dse")))

    @needs_xpurt
    def test_profile_links_land_where_the_loader_globs(self) -> None:
        ws = xpurt.build_workspace()
        relevant = [
            p for p in xpurt.discover_profiles()
            if p.target == "firesim_rocket_saturn"
        ]
        self.assertTrue(xpurt.link_profiles(ws, relevant) >= 6)
        for hw in ("gemmini_q31", "V256D128_rvv"):
            for model, basename in (
                ("dronet", "dronet.int8"),
                ("yolov8_nano", "yolov8_nano.int8"),
                ("mlp_control", "mlp_control.fp32"),
            ):
                target = ws / "gen" / "profile" / hw / "firesim_rocket_saturn" / model / basename
                with self.subTest(hw=hw, model=model):
                    matches = list(target.glob("*/results.csv")) + list(
                        target.glob("*/*/results.csv")
                    )
                    self.assertTrue(matches, f"loader would find nothing under {target}")

    @needs_xpurt
    def test_linking_is_idempotent(self) -> None:
        ws = xpurt.build_workspace()
        relevant = [p for p in xpurt.discover_profiles() if p.target == "firesim_rocket_saturn"]
        first = xpurt.link_profiles(ws, relevant)
        second = xpurt.link_profiles(ws, relevant)
        self.assertEqual(first, second)


class SummaryTests(unittest.TestCase):
    SCHEDULE = {
        "metadata": {
            "makespan": 57.024,
            "num_operations": 4,
            "machines": ["CPU_P#0", "CPU_E#0"],
            "profile_hw": {"CPU_P": "gemmini_q31", "CPU_E": "V256D128_rvv"},
        },
        "dispatches": {
            "0": {"job_name": "mlp_control0", "hardware_target": "CPU_P#0",
                  "start_time": 0.0, "duration": 0.5},
            "1": {"job_name": "mlp_control1", "hardware_target": "CPU_P#0",
                  "start_time": 20.0, "duration": 0.5},
            "2": {"job_name": "dronet0", "hardware_target": "CPU_E#0",
                  "start_time": 1.0, "duration": 4.0},
            "3": {"job_name": "dronet0", "hardware_target": "CPU_P#0",
                  "start_time": 5.0, "duration": 1.0},
        },
    }

    def _summary(self):
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "s.json"
            path.write_text(json.dumps(self.SCHEDULE))
            return xpurt.summarize(path)

    def test_makespan_and_op_count_come_from_metadata(self) -> None:
        s = self._summary()
        self.assertAlmostEqual(s["makespan"], 57.024)
        self.assertEqual(s["num_operations"], 4)

    def test_job_latency_spans_all_its_dispatches(self) -> None:
        s = self._summary()
        # dronet0 runs 1.0->5.0 on CPU_E then 5.0->6.0 on CPU_P: latency 5.0.
        self.assertEqual(s["jobs"]["dronet0"]["latency"], 5.0)
        self.assertEqual(s["jobs"]["dronet0"]["dispatches"], 2)
        self.assertEqual(s["jobs"]["dronet0"]["targets"], ["CPU_E#0", "CPU_P#0"])

    def test_periodic_instances_collapse_to_one_network(self) -> None:
        s = self._summary()
        self.assertEqual(s["networks"]["mlp_control"]["instances"], 2)
        self.assertEqual(s["networks"]["mlp_control"]["worst_latency"], 0.5)
        self.assertEqual(s["networks"]["dronet"]["instances"], 1)

    def test_cycles_are_absent_until_a_clock_is_named(self) -> None:
        """A millisecond with no stated frequency has no cycle count to report."""
        s = self._summary()
        self.assertIsNone(s["makespan_cycles"])
        self.assertIsNone(s["jobs"]["dronet0"]["latency_cycles"])

    def test_cycles_are_reported_alongside_wall_time(self) -> None:
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "s.json"
            path.write_text(json.dumps(self.SCHEDULE))
            s = xpurt.summarize(path, clock_mhz=50.0)
        # 5.0 ms at 50 MHz is 250,000 cycles.
        self.assertEqual(s["jobs"]["dronet0"]["latency_cycles"], 250_000)
        self.assertEqual(s["networks"]["dronet"]["worst_latency_cycles"], 250_000)
        self.assertEqual(s["makespan_cycles"], round(57.024 * 50_000))


class RetimeTests(unittest.TestCase):
    """Cycles are the hardware-independent quantity; wall time is derived once."""

    HEADER = "dispatch_id,module_name,mean_time,mean_unit,mean_time_ns,cycles\n"

    def _csv(self, directory: Path, rows: str) -> Path:
        path = directory / "results.csv"
        path.write_text(self.HEADER + rows)
        return path

    def test_wall_time_is_recomputed_from_cycles(self) -> None:
        with TemporaryDirectory() as tmp:
            src = self._csv(Path(tmp), "0,conv,1.910402,ms,1910402.000000,1910402\n")
            dest = Path(tmp) / "out" / "results.csv"
            report = xpurt.retime_results_csv(src, dest, clock_mhz=50.0)
            with dest.open() as handle:
                rows = list(csv.DictReader(handle))
        self.assertEqual(report.converted, 1)
        self.assertEqual(report.passthrough, 0)
        # 1,910,402 cycles at 50 MHz is 38.208040 ms, not the 1.910402 ms it
        # was captured as at 1 GHz.
        self.assertAlmostEqual(float(rows[0]["mean_time"]), 38.20804, places=5)
        self.assertEqual(rows[0]["mean_unit"], "ms")
        self.assertEqual(rows[0]["cycles"], "1910402")

    def test_capture_clock_is_derived_from_the_data(self) -> None:
        """cycles / mean_time_ns is the frequency, and it beats asserting 1 GHz."""
        with TemporaryDirectory() as tmp:
            src = self._csv(
                Path(tmp),
                "0,a,1.000000,ms,1000000.000000,1000000\n"
                "1,b,2.000000,ms,2000000.000000,2000000\n",
            )
            report = xpurt.retime_results_csv(src, Path(tmp) / "o" / "results.csv", 50.0)
        self.assertEqual(report.capture_mhz, 1000.0)
        self.assertTrue(report.capture_consistent)

    def test_rows_without_cycles_are_counted_not_hidden(self) -> None:
        """A partial conversion must never pass for a complete one."""
        with TemporaryDirectory() as tmp:
            src = self._csv(
                Path(tmp),
                "0,a,1.000000,ms,1000000.000000,1000000\n"
                "1,b,5.000000,ms,5000000.000000,\n",
            )
            dest = Path(tmp) / "o" / "results.csv"
            report = xpurt.retime_results_csv(src, dest, 50.0)
            with dest.open() as handle:
                rows = list(csv.DictReader(handle))
        self.assertEqual(report.converted, 1)
        self.assertEqual(report.passthrough, 1)
        self.assertFalse(report.complete)
        # The unconvertible row keeps its original time rather than being dropped.
        self.assertAlmostEqual(float(rows[1]["mean_time"]), 5.0)

    def test_a_profile_without_a_cycles_column_is_refused(self) -> None:
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "results.csv"
            path.write_text("dispatch_id,mean_time,mean_unit\n0,1.0,ms\n")
            self.assertIsNone(
                xpurt.retime_results_csv(path, Path(tmp) / "o" / "results.csv", 50.0)
            )

    def test_a_nonpositive_clock_is_refused(self) -> None:
        with TemporaryDirectory() as tmp:
            src = self._csv(Path(tmp), "0,a,1.0,ms,1000000.0,1000000\n")
            with self.assertRaises(xpurt.XpurtError):
                xpurt.retime_results_csv(src, Path(tmp) / "o" / "results.csv", 0.0)

    def test_real_profiles_retime_completely(self) -> None:
        """All six flight profiles carry cycles for every op, at exactly 1 GHz."""
        with TemporaryDirectory() as tmp:
            ws = Path(tmp)
            relevant = [
                p for p in xpurt.discover_profiles()
                if p.target == "firesim_rocket_saturn"
            ]
            reports, unconverted = xpurt.retime_profiles(ws, relevant, 50.0)
        self.assertEqual(unconverted, [])
        self.assertTrue(reports)
        for report in reports:
            with self.subTest(profile=report.profile.key):
                self.assertTrue(report.complete, "some ops carry no cycle count")
                self.assertEqual(report.capture_mhz, 1000.0)
                self.assertTrue(report.capture_consistent)

    @needs_xpurt
    def test_retimed_profiles_land_where_the_loader_globs(self) -> None:
        with TemporaryDirectory() as tmp:
            ws = Path(tmp)
            relevant = [
                p for p in xpurt.discover_profiles()
                if p.target == "firesim_rocket_saturn"
            ]
            xpurt.retime_profiles(ws, relevant, 50.0)
            for hw in ("gemmini_q31", "V256D128_rvv"):
                for model, basename in (
                    ("dronet", "dronet.int8"),
                    ("yolov8_nano", "yolov8_nano.int8"),
                    ("mlp_control", "mlp_control.fp32"),
                ):
                    target = (
                        ws / "gen" / "profile" / hw / "firesim_rocket_saturn"
                        / model / basename
                    )
                    with self.subTest(hw=hw, model=model):
                        self.assertTrue(list(target.glob("*/*/results.csv")))

    def test_retiming_writes_only_inside_the_mirror(self) -> None:
        """XPU-RT's gen/ tree is tracked; retiming must not touch the source."""
        relevant = [
            p for p in xpurt.discover_profiles() if p.target == "firesim_rocket_saturn"
        ]
        before = {p.results: p.results.read_bytes() for p in relevant}
        with TemporaryDirectory() as tmp:
            xpurt.retime_profiles(Path(tmp), relevant, 50.0)
        for path, content in before.items():
            with self.subTest(path=path):
                self.assertEqual(path.read_bytes(), content)

    def test_linking_after_retiming_discards_the_retimed_data(self) -> None:
        """Otherwise a pass-through run silently reports the previous clock.

        `rb xpurt run --config X` retimes into the mirror; a later run without a
        clock asks for the capture-clock numbers. If the retimed directory
        survived, those stale times would be reported as if they were captured.
        """
        with TemporaryDirectory() as tmp:
            ws = Path(tmp)
            relevant = [
                p for p in xpurt.discover_profiles()
                if p.target == "firesim_rocket_saturn"
            ]
            xpurt.retime_profiles(ws, relevant, 50.0)
            self.assertEqual(xpurt.link_profiles(ws, relevant), len(relevant))
            for prof in relevant:
                dest = (
                    ws / "gen" / "profile" / prof.hw / prof.target
                    / prof.model / prof.basename
                )
                with self.subTest(key=prof.key):
                    self.assertTrue(dest.is_symlink())

    def test_derive_capture_clock_agrees_with_retiming(self) -> None:
        for prof in xpurt.discover_profiles():
            if prof.target != "firesim_rocket_saturn":
                continue
            with self.subTest(key=prof.key):
                self.assertEqual(xpurt.derive_capture_clock(prof), 1000.0)


class ProfileTargetTests(unittest.TestCase):
    @needs_profile_targets
    def test_the_firesim_target_is_declared_not_extracted(self) -> None:
        got = xpurt.load_profile_target("firesim_rocket_saturn")
        self.assertIsNotNone(got)
        self.assertEqual(got.evidence, "declared")

    def test_an_unknown_target_has_no_descriptor(self) -> None:
        self.assertIsNone(xpurt.load_profile_target("no_such_target"))


class ForestReplayTests(unittest.TestCase):
    @needs_forest_schedule
    def test_tracked_demo_schedule_has_expected_replay_contract(self) -> None:
        summary = xpurt.forest_schedule_summary(xpurt.FOREST_SCHEDULE)
        self.assertGreater(summary["period_ms"], 0.0)
        self.assertGreater(summary["operations"], 0)
        self.assertIn("dronet", summary["models"])
        self.assertIn("yolov8_nano", summary["models"])
        self.assertIn("input sampled at job start", summary["timing_semantics"])

    @needs_forest_schedule
    def test_forest_plot_labels_schedule_timing_as_replayed(self) -> None:
        summary = xpurt.forest_schedule_summary(xpurt.FOREST_SCHEDULE)
        with TemporaryDirectory() as tmp:
            output = Path(tmp) / "forest.svg"
            xpurt.render_forest_plot(summary, output)
            text = output.read_text(encoding="utf-8")
        self.assertIn("host-CUDA inference", text)
        self.assertIn("not measurements", text)
        self.assertIn("dronet", text)

    def test_offscreen_pilot_adapter_is_tracked(self) -> None:
        self.assertTrue(xpurt.FOREST_LAUNCHER.is_file())
        text = xpurt.FOREST_LAUNCHER.read_text(encoding="utf-8")
        self.assertIn("AppLauncher", text)
        self.assertIn("offscreen_video", text)


if __name__ == "__main__":
    unittest.main()
