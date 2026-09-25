#!/usr/bin/env python3

import copy
import json
import pathlib
import subprocess
import sys
import tempfile


ROOT = pathlib.Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests" / "fixtures"
BINARY = pathlib.Path(
    sys.argv[1] if len(sys.argv) > 1 else ROOT / "bin" / "vroom"
).resolve()


def solve(data):
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json") as input_file:
        json.dump(data, input_file)
        input_file.flush()
        return subprocess.run(
            [str(BINARY), "-i", input_file.name],
            check=False,
            capture_output=True,
            text=True,
        )


def output_for(data):
    result = solve(data)
    assert result.returncode == 0, result.stderr
    return json.loads(result.stdout)


def assigned_ids(output):
    return {
        step.get("id")
        for route in output["routes"]
        for step in route["steps"]
        if step.get("type") in {"job", "pickup", "delivery"}
    }


def assert_starts_within_windows(output, jobs):
    windows = {}
    for job in jobs:
        if "id" in job:
            windows[job["id"]] = job.get("time_windows", [])
    for shipment in jobs:
        if "pickup" in shipment:
            for role in ("pickup", "delivery"):
                item = shipment[role]
                windows[item["id"]] = item.get("time_windows", [])
    for route in output["routes"]:
        for step in route["steps"]:
            if step.get("type") not in {"job", "pickup", "delivery"}:
                continue
            start = step["arrival"] + step["waiting_time"]
            assert any(begin <= start <= end for begin, end in windows[step["id"]]), (
                step,
                windows[step["id"]],
            )


def main():
    shipment = json.loads((FIXTURES / "issue_1355_shipment.json").read_text())
    jobs_with_steps = json.loads(
        (FIXTURES / "issue_1355_jobs_with_steps.json").read_text()
    )

    output = output_for(shipment)
    assert not output["unassigned"]
    assert assigned_ids(output) == {1, 2}
    assert_starts_within_windows(output, shipment["shipments"])

    output = output_for(jobs_with_steps)
    assert not output["unassigned"]
    assert assigned_ids(output) == {1, 2}
    steps = output["routes"][0]["steps"]
    assert [step.get("id") for step in steps if step["type"] == "job"] == [1, 2]
    assert steps.index(next(step for step in steps if step["type"] == "break")) > max(
        steps.index(step) for step in steps if step["type"] == "job"
    )
    assert_starts_within_windows(output, jobs_with_steps["jobs"])

    no_break = copy.deepcopy(shipment)
    no_break["vehicles"][0].pop("breaks")
    output = output_for(no_break)
    assert not output["unassigned"]
    assert assigned_ids(output) == {1, 2}
    assert all(step["type"] != "break" for step in output["routes"][0]["steps"])

    break_before_pickup = copy.deepcopy(shipment)
    break_before_pickup["vehicles"][0]["breaks"][0]["time_windows"] = [[36000, 42000]]
    break_before_pickup["shipments"][0]["pickup"]["time_windows"] = [[46800, 50000]]
    break_before_pickup["shipments"][0]["delivery"]["time_windows"] = [[52000, 54000]]
    output = output_for(break_before_pickup)
    assert not output["unassigned"]
    steps = output["routes"][0]["steps"]
    assert steps.index(next(step for step in steps if step["type"] == "break")) < steps.index(
        next(step for step in steps if step["type"] == "pickup")
    )
    assert_starts_within_windows(output, break_before_pickup["shipments"])

    impossible_delivery = copy.deepcopy(shipment)
    impossible_delivery["shipments"][0]["delivery"]["time_windows"] = [[40000, 40000]]
    output = output_for(impossible_delivery)
    assert {1, 2} <= {item["id"] for item in output["unassigned"]}

    output = output_for(json.loads((ROOT / "docs" / "example_2.json").read_text()))
    expected = json.loads((ROOT / "docs" / "example_2_sol.json").read_text())
    output["summary"].pop("computing_times", None)
    assert output == expected

    print("issue #1355 regression and controls passed")


if __name__ == "__main__":
    main()
