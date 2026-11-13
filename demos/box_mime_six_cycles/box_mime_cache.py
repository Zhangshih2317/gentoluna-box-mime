"""Reuse a fixed sampled path only while the model, code and targets match."""

import hashlib
import json
import math
from pathlib import Path

from box_motion_plan import collision_samples, delta
from box_motion_robot import Geometry, SDK_ROOT, emit
from box_mime_taught_plan import build_stages, NATIVE_ARM_POSITION
from native_arm_motion import ENVELOPE_MODEL, ENVELOPE_GRID_DEG, is_native_arm_stage


def model_fingerprint(geometry):
    import collision_guard
    model = geometry.checker.model
    root = Path(collision_guard.__file__).resolve().parent
    paths = {model.urdf_path, geometry.checker.exact_backend.library_path}
    paths.update(Path(g.mesh_path) for g in model.geometries if g.mesh_path)
    paths.update(root.rglob("*.py"))
    paths.update(root.rglob("*.json"))
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}


def main_stages(stages):
    return [s for s in stages if s["name"] not in (
        "align_arms_home", "align_standing_home", "align_body_from_crouch",
        "align_waist_from_turn")]


def verify_binding(certificate, code_id, fingerprint, stages):
    if (not certificate or certificate.get("schema") != "luna_mime_geometry_cache_v1"
            or certificate["code_id"] != code_id
            or certificate["model_files"] != fingerprint):
        raise RuntimeError("Geometry cache missing or changed; prepare cache offline first")
    expected, actual = certificate["stages"], main_stages(stages)
    if NATIVE_ARM_POSITION:
        report = certificate["report"]
        if (report.get("native_arm_envelope_model") != ENVELOPE_MODEL
                or report.get("native_arm_envelope_step_deg") != ENVELOPE_GRID_DEG
                or report.get("native_arm_envelopes_checked") !=
                [s["name"] for s in actual if is_native_arm_stage(s)]):
            raise RuntimeError("Native arm envelope cache missing or changed")
    if len(expected) != len(actual):
        raise RuntimeError("Cached stage count differs")
    for old, new in zip(expected, actual):
        if old["name"] != new["name"] or old["moving"] != new["moving"]:
            raise RuntimeError("Cached motion order differs")
        # Only encoder rounding in the held HEAD is tolerated. No arm/body
        # target changes are covered by the cached main motion.
        for endpoint in ("start", "target"):
            if delta(old[endpoint], new[endpoint], ("arm0", "arm1", "body")) > 1e-9:
                raise RuntimeError("Cached arm/body targets differ")
            if delta(old[endpoint], new[endpoint], ("head",)) > .002:
                raise RuntimeError("Held head differs from cached path")


def fast_check(geometry, robot, stages, certificate, code_id):
    verify_binding(certificate, code_id, model_fingerprint(geometry), stages)
    geometry.verify_sdk(robot, stages)
    current_flange = {str(k): v for k, v in geometry.flange_transforms.items()}
    if current_flange != certificate.get("flange_transforms"):
        raise RuntimeError("SDK flange configuration differs from cached geometry")
    count = 0
    for stage in stages:
        if stage["name"] in ("align_arms_home", "align_standing_home", "align_body_from_crouch",
                             "align_waist_from_turn"):
            for sample in collision_samples(stage, .1):
                geometry.check(sample)
                count += 1
    emit("geometry_cache_reused", main_path_samples=certificate["report"]["samples"],
         home_path_samples=count, minimum_main_flange_spacing_mm=
         certificate["report"]["minimum_flange_spacing_mm"])


def prepare(home, squat, waist, frame_parameters, code_id):
    """Model-only work. No SDK connection, mode changes, or motion commands."""
    from box_mime_taught_robot import check_geometry
    geometry = Geometry()
    try:
        before = model_fingerprint(geometry)
        for arm in (0, 1):
            values = frame_parameters[str(arm)]
            alpha, theta = (math.radians(values["FLANGE."+k]) for k in ("Alpha", "Theta"))
            ca, sa, ct, st = math.cos(alpha), math.sin(alpha), math.cos(theta), math.sin(theta)
            a, d = values["FLANGE.A"], values["FLANGE.D"]
            geometry.flange_transforms[arm] = [
                [ct, -st, 0, a], [ca*st, ca*ct, -sa, -d*sa],
                [sa*st, sa*ct, ca, d*ca], [0, 0, 0, 1]]
        geometry.verify_sdk = lambda *_: emit("offline_geometry_only",
                                             live_vendor_fk_required_on_start=True)
        stages = build_stages(home, home, squat, waist)
        report = check_geometry(geometry, None, stages)
        if model_fingerprint(geometry) != before:
            raise RuntimeError("Model changed while building cache")
        certificate = {"schema": "luna_mime_geometry_cache_v1", "code_id": code_id,
                       "model_files": before, "stages": stages, "report": report,
                       "flange_transforms": {str(k): v for k, v in geometry.flange_transforms.items()},
                       "motion_commands_sent": 0}
        emit("geometry_cache_prepared", certificate=certificate)
        return certificate
    finally:
        geometry.close()
