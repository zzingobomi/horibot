"""TASK_REGISTRY 의 TaskDefinition + required_capabilities 자체 자리 자체 자리.

frontend TasksPage 의 robot dropdown filter 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리
— /tasks endpoint 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리
required_capabilities 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 contract.
"""

from __future__ import annotations

from nodes.application.task_node import TASK_REGISTRY


def test_task_registry_has_pick_and_place_and_scan():
    assert "pick_and_place" in TASK_REGISTRY
    assert "scan" in TASK_REGISTRY


def test_pick_and_place_no_required_capabilities():
    """pick_and_place 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 robot 자체 자리 자체 자리 자체 자리."""
    defn = TASK_REGISTRY["pick_and_place"]
    assert defn.required_capabilities == ()


def test_scan_requires_rgbd():
    """ScanTask 자체 자리 자체 자리 자체 자리 자체 자리 rgbd capability 자체 자리 자체 자리 자체 자리 자체 자리."""
    defn = TASK_REGISTRY["scan"]
    assert defn.required_capabilities == ("rgbd",)


def test_scan_factory_builds_steps():
    """create_scan_task 자체 자리 자체 자리 자체 자리 NewSession → ForEach(MoveJ + CaptureScan)
    → BuildReconstruction 자체 자리 자체 자리 자체 자리 step 시퀀스 자체 자리 자체 자리."""
    from modules.task.steps import (
        BuildReconstruction,
        CaptureScan,
        ForEach,
        MoveJByName,
        NewSession,
    )

    defn = TASK_REGISTRY["scan"]
    task = defn.factory({"label": "test", "scan_poses": ["home"]})

    assert task.name == "scan"
    assert len(task.steps) == 3
    assert isinstance(task.steps[0], NewSession)
    assert isinstance(task.steps[1], ForEach)
    assert isinstance(task.steps[2], BuildReconstruction)

    # ForEach body 자체 자리 자체 자리 자체 자리 자체 자리 — MoveJByName + CaptureScan
    inner = task.steps[1]
    assert len(inner.children) == 2
    assert isinstance(inner.children[0], MoveJByName)
    assert isinstance(inner.children[1], CaptureScan)

    # CaptureScan.session 자체 자리 자체 자리 자체 자리 NewSession.out (typed Slot)
    capture = inner.children[1]
    assert capture.session.step_id == task.steps[0].id

    # BuildReconstruction.session 자체 자리 자체 자리 자체 자리 자체 자리 NewSession.out
    build = task.steps[2]
    assert build.session.step_id == task.steps[0].id
