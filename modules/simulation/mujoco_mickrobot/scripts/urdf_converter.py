"""Strict offline conversion from MickRobot URDF to a complete MuJoCo model."""

from __future__ import annotations

import copy
import hashlib
import math
from pathlib import Path
import struct
import shutil
import tempfile
import xml.etree.ElementTree as ET

import numpy as np


STL_FACE_LIMIT = 200_000
STL_FACE_TARGET = 190_000


class ConversionError(RuntimeError):
    """Raised when conversion input cannot produce a valid MuJoCo model."""


def _numbers(text: str | None, count: int, context: str, default: str | None = None) -> tuple[float, ...]:
    source = default if text is None else text
    try:
        values = tuple(float(value) for value in (source or "").split())
    except ValueError as exc:
        raise ConversionError(f"{context} must contain {count} finite numbers") from exc
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise ConversionError(f"{context} must contain {count} finite numbers")
    return values


def _fmt(values: tuple[float, ...]) -> str:
    return " ".join(f"{value:.12g}" for value in values)


def _rpy_quat(rpy: tuple[float, float, float]) -> tuple[float, float, float, float]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _rpy_matrix(rpy: tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )
    )


def _pose(element: ET.Element | None, context: str) -> dict[str, str]:
    if element is None:
        return {"pos": "0 0 0", "quat": "1 0 0 0"}
    xyz = _numbers(element.get("xyz"), 3, f"{context}.xyz", "0 0 0")
    rpy = _numbers(element.get("rpy"), 3, f"{context}.rpy", "0 0 0")
    return {"pos": _fmt(xyz), "quat": _fmt(_rpy_quat(rpy))}


def _load_xml(path: Path, expected_root: str) -> ET.Element:
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise ConversionError(f"cannot load {path}: {exc}") from exc
    if root.tag != expected_root:
        raise ConversionError(f"{path} root must be <{expected_root}>")
    return root


def _reduce_stl(source: Path, destination: Path) -> Path:
    payload = source.read_bytes()
    if len(payload) < 84:
        raise ConversionError(f"STL file is too short: {source}")
    face_count = struct.unpack_from("<I", payload, 80)[0]
    if 84 + face_count * 50 != len(payload):
        raise ConversionError(f"only binary STL is supported: {source}")
    if face_count <= STL_FACE_LIMIT:
        return source
    selected = bytearray(payload[:80])
    selected.extend(struct.pack("<I", STL_FACE_TARGET))
    for output_index in range(STL_FACE_TARGET):
        source_index = output_index * face_count // STL_FACE_TARGET
        offset = 84 + source_index * 50
        selected.extend(payload[offset : offset + 50])
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=destination.parent, prefix=source.stem + "-", suffix=source.suffix, delete=False) as temporary:
        temporary.write(selected)
        temporary_path = Path(temporary.name)
    temporary_path.replace(destination)
    return destination


def _geometry(
    geometry: ET.Element,
    asset: ET.Element,
    assets: dict[tuple[str, str, bool], str],
    urdf_dir: Path,
    generated_assets: Path,
    *,
    reduce_visual: bool,
) -> dict[str, str]:
    child = next(iter(geometry), None)
    if child is None:
        raise ConversionError("URDF geometry is empty")
    if child.tag == "box":
        size = _numbers(child.get("size"), 3, "box.size")
        return {"type": "box", "size": _fmt(tuple(value / 2 for value in size))}
    if child.tag == "sphere":
        radius = _numbers(child.get("radius"), 1, "sphere.radius")[0]
        return {"type": "sphere", "size": f"{radius:.12g}"}
    if child.tag == "cylinder":
        radius = _numbers(child.get("radius"), 1, "cylinder.radius")[0]
        length = _numbers(child.get("length"), 1, "cylinder.length")[0]
        return {"type": "cylinder", "size": _fmt((radius, length / 2))}
    if child.tag != "mesh":
        raise ConversionError(f"unsupported URDF geometry: {child.tag}")
    filename = child.get("filename")
    if not filename:
        raise ConversionError("URDF mesh filename is missing")
    source = (urdf_dir / filename).resolve()
    if not source.is_file():
        raise ConversionError(f"URDF mesh does not exist: {source}")
    scale = _numbers(child.get("scale"), 3, f"mesh {filename} scale", "1 1 1")
    key = (str(source), _fmt(scale), reduce_visual)
    if key not in assets:
        name = f"mesh_{len(assets)}_{source.stem}"
        digest = hashlib.sha256(source.read_bytes()).hexdigest()[:16]
        destination = generated_assets / f"{source.stem}-{digest}{source.suffix}"
        usable = _reduce_stl(source, destination) if reduce_visual else source
        ET.SubElement(asset, "mesh", name=name, file=str(usable), scale=_fmt(scale))
        assets[key] = name
    return {"type": "mesh", "mesh": assets[key]}


def _add_link_contents(
    body: ET.Element,
    link: ET.Element,
    asset: ET.Element,
    assets: dict[tuple[str, str, bool], str],
    urdf_dir: Path,
    generated_assets: Path,
) -> None:
    name = link.get("name", "")
    inertial = link.find("inertial")
    if inertial is not None:
        mass_element = inertial.find("mass")
        inertia = inertial.find("inertia")
        if mass_element is None or inertia is None:
            raise ConversionError(f"link {name} inertial requires mass and inertia")
        mass = _numbers(mass_element.get("value"), 1, f"link {name} mass")[0]
        ixx, iyy, izz, ixy, ixz, iyz = tuple(
            _numbers(inertia.get(key), 1, f"link {name} inertia.{key}")[0]
            for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
        )
        origin = inertial.find("origin")
        xyz = _numbers(origin.get("xyz") if origin is not None else None, 3, f"link {name} inertial origin.xyz", "0 0 0")
        rpy = _numbers(origin.get("rpy") if origin is not None else None, 3, f"link {name} inertial origin.rpy", "0 0 0")
        inertia_matrix = np.array(((ixx, ixy, ixz), (ixy, iyy, iyz), (ixz, iyz, izz)))
        rotation = _rpy_matrix(rpy)  # type: ignore[arg-type]
        rotated = rotation @ inertia_matrix @ rotation.T
        values = (rotated[0, 0], rotated[1, 1], rotated[2, 2], rotated[0, 1], rotated[0, 2], rotated[1, 2])
        ET.SubElement(body, "inertial", mass=f"{mass:.12g}", fullinertia=_fmt(tuple(float(value) for value in values)), pos=_fmt(xyz))
    for index, visual in enumerate(link.findall("visual")):
        geometry = visual.find("geometry")
        if geometry is None:
            raise ConversionError(f"link {name} visual has no geometry")
        attributes = _geometry(geometry, asset, assets, urdf_dir, generated_assets, reduce_visual=True)
        attributes.update(_pose(visual.find("origin"), f"link {name} visual origin"))
        attributes.update({"name": visual.get("name", f"{name}_visual_{index}")})
        material = visual.find("material")
        color = material.find("color") if material is not None else None
        if color is not None and color.get("rgba") is not None:
            attributes["rgba"] = _fmt(_numbers(color.get("rgba"), 4, f"link {name} visual color"))
        ET.SubElement(body, "geom", **attributes)
    for index, collision in enumerate(link.findall("collision")):
        geometry = collision.find("geometry")
        if geometry is None:
            raise ConversionError(f"link {name} collision has no geometry")
        attributes = _geometry(geometry, asset, assets, urdf_dir, generated_assets, reduce_visual=False)
        attributes.update(_pose(collision.find("origin"), f"link {name} collision origin"))
        attributes.update({"name": collision.get("name", f"{name}_collision_{index}")})
        ET.SubElement(body, "geom", **attributes)


def _apply_extensions(root: ET.Element, extension: ET.Element, bodies: dict[str, ET.Element], joints: dict[str, ET.Element]) -> None:
    for kind in ("visual", "collision"):
        defaults = extension.find(kind)
        if defaults is None:
            raise ConversionError(f"MuJoCo extension requires <{kind}> defaults")
        suffix = "_visual_" if kind == "visual" else "_collision_"
        for geom in root.findall(".//geom"):
            if suffix in geom.get("name", ""):
                geom.attrib.update(defaults.attrib)
    for addition in extension.findall("body"):
        target = addition.get("target", "")
        if target not in bodies:
            raise ConversionError(f"MuJoCo extension target link does not exist: {target}")
        for child in addition:
            bodies[target].append(copy.deepcopy(child))
    for addition in extension.findall("joint"):
        target = addition.get("target", "")
        if target not in joints:
            raise ConversionError(f"MuJoCo extension target joint does not exist: {target}")
        for key, value in addition.attrib.items():
            if key != "target":
                joints[target].set(key, value)
    geoms = {element.get("name", ""): element for element in root.findall(".//geom")}
    for addition in extension.findall("geom"):
        target = addition.get("target", "")
        if target not in geoms:
            raise ConversionError(f"MuJoCo extension target geom does not exist: {target}")
        for key, value in addition.attrib.items():
            if key != "target":
                geoms[target].set(key, value)
    for section_name in ("actuator", "sensor"):
        section = extension.find(section_name)
        if section is not None:
            root.append(copy.deepcopy(section))


def convert_urdf(urdf_path: Path, extension_path: Path, world_path: Path, output_path: Path) -> Path:
    """Convert inputs, validate with MuJoCo, and atomically publish output_path."""
    urdf_path, extension_path, world_path, output_path = map(Path, (urdf_path, extension_path, world_path, output_path))
    urdf = _load_xml(urdf_path, "robot")
    extension = _load_xml(extension_path, "mujoco_extensions")
    world_template = _load_xml(world_path, "mujoco")
    link_elements = urdf.findall("link")
    joint_elements = urdf.findall("joint")
    link_names = [link.get("name", "") for link in link_elements]
    joint_names = [joint.get("name", "") for joint in joint_elements]
    for names, kind in ((link_names, "link"), (joint_names, "joint")):
        duplicates = sorted({name for name in names if names.count(name) > 1})
        if duplicates:
            raise ConversionError(f"duplicate URDF {kind} name: {duplicates[0]}")
    links = dict(zip(link_names, link_elements))
    urdf_joints = dict(zip(joint_names, joint_elements))
    if "" in links or "" in urdf_joints:
        raise ConversionError("all URDF links and joints require names")
    child_joints: dict[str, list[ET.Element]] = {name: [] for name in links}
    children: set[str] = set()
    for name, joint in urdf_joints.items():
        parent_element, child_element = joint.find("parent"), joint.find("child")
        parent = parent_element.get("link", "") if parent_element is not None else ""
        child = child_element.get("link", "") if child_element is not None else ""
        if parent not in links or child not in links:
            raise ConversionError(f"joint {name} references missing parent or child link")
        child_joints[parent].append(joint)
        if child in children:
            raise ConversionError(f"link has multiple parent joints: {child}")
        children.add(child)
    roots = sorted(set(links) - children)
    if len(roots) != 1:
        raise ConversionError(f"URDF must have exactly one root link, got: {roots}")

    model = ET.Element("mujoco", model=urdf.get("name", "mickrobot"))
    ET.SubElement(model, "compiler", angle="radian", meshdir="/", autolimits="true", inertiafromgeom="false", balanceinertia="true")
    ET.SubElement(model, "option", timestep="0.002", gravity="0 0 -9.80665", integrator="implicitfast")
    asset = ET.SubElement(model, "asset")
    worldbody = ET.SubElement(model, "worldbody")
    ET.SubElement(worldbody, "light", name="sun", pos="0 0 8", dir="0 0 -1", directional="true")
    template_world = world_template.find("worldbody")
    if template_world is None:
        raise ConversionError("test world has no worldbody")
    for child in template_world:
        worldbody.append(copy.deepcopy(child))
    bodies: dict[str, ET.Element] = {}
    output_path.parent.mkdir(parents=True, exist_ok=True)
    staging_context = tempfile.TemporaryDirectory(dir=output_path.parent, prefix="mickrobot-stage-")
    staging_dir = Path(staging_context.name)
    generated_assets = staging_dir / "assets"
    assets: dict[tuple[str, str, bool], str] = {}
    mj_joints: dict[str, ET.Element] = {}
    urdf_limit_metadata: list[tuple[str, str]] = []
    visited_urdf_joints: set[str] = set()

    def add_body(link_name: str, parent_xml: ET.Element, joint: ET.Element | None) -> None:
        attributes = {"name": link_name}
        if joint is not None:
            attributes.update(_pose(joint.find("origin"), f"joint {joint.get('name')} origin"))
        body = ET.SubElement(parent_xml, "body", **attributes)
        bodies[link_name] = body
        _add_link_contents(body, links[link_name], asset, assets, urdf_path.parent, generated_assets)
        if joint is not None and joint.get("type") != "fixed":
            joint_name = joint.get("name", "")
            visited_urdf_joints.add(joint_name)
            joint_type = joint.get("type")
            if joint_type not in {"continuous", "revolute"}:
                raise ConversionError(f"unsupported joint type for {joint_name}: {joint_type}")
            axis_element = joint.find("axis")
            axis = _numbers(axis_element.get("xyz") if axis_element is not None else None, 3, f"joint {joint_name} axis", "1 0 0")
            mj_joint = ET.SubElement(body, "joint", name=joint_name, type="hinge", axis=_fmt(axis))
            dynamics = joint.find("dynamics")
            if dynamics is not None:
                if dynamics.get("damping") is not None:
                    mj_joint.set("damping", f"{_numbers(dynamics.get('damping'), 1, f'joint {joint_name} damping')[0]:.12g}")
                if dynamics.get("friction") is not None:
                    mj_joint.set("frictionloss", f"{_numbers(dynamics.get('friction'), 1, f'joint {joint_name} friction')[0]:.12g}")
            limit = joint.find("limit")
            if limit is not None:
                for field in ("effort", "velocity"):
                    if limit.get(field) is not None:
                        value = _numbers(limit.get(field), 1, f"joint {joint_name} {field}")[0]
                        urdf_limit_metadata.append((f"urdf_limit.{joint_name}.{field}", f"{value:.12g}"))
            if joint_type == "revolute":
                if limit is None or limit.get("lower") is None or limit.get("upper") is None:
                    raise ConversionError(f"revolute joint {joint_name} requires lower and upper limits")
                lower = _numbers(limit.get("lower"), 1, f"joint {joint_name} lower")[0]
                upper = _numbers(limit.get("upper"), 1, f"joint {joint_name} upper")[0]
                if lower >= upper:
                    raise ConversionError(f"joint {joint_name} lower limit must be less than upper limit")
                mj_joint.set("limited", "true")
                mj_joint.set("range", _fmt((lower, upper)))
            mj_joints[joint_name] = mj_joint
        elif joint is not None:
            visited_urdf_joints.add(joint.get("name", ""))
        for child_joint in child_joints[link_name]:
            child_element = child_joint.find("child")
            add_body(child_element.get("link", ""), body, child_joint)  # type: ignore[union-attr]

    add_body(roots[0], worldbody, None)
    if set(bodies) != set(links) or visited_urdf_joints != set(urdf_joints):
        missing_links = sorted(set(links) - set(bodies))
        missing_joints = sorted(set(urdf_joints) - visited_urdf_joints)
        raise ConversionError(f"disconnected URDF resources: links={missing_links}, joints={missing_joints}")
    _apply_extensions(model, extension, bodies, mj_joints)
    if urdf_limit_metadata:
        custom = ET.SubElement(model, "custom")
        for name, value in urdf_limit_metadata:
            ET.SubElement(custom, "numeric", name=name, data=value)
    ET.indent(model, space="  ")
    payload = ET.tostring(model, encoding="utf-8", xml_declaration=True)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=staging_dir, prefix="mickrobot-", suffix=".xml", delete=False) as temporary:
        temporary.write(payload)
        temporary_path = Path(temporary.name)
    try:
        import mujoco

        mujoco.MjModel.from_xml_path(str(temporary_path))
    except Exception as exc:
        temporary_path.unlink(missing_ok=True)
        raise ConversionError(f"generated MuJoCo model is invalid: {exc}") from exc
    final_assets = output_path.parent / "assets"
    if generated_assets.exists():
        final_assets.mkdir(parents=True, exist_ok=True)
        for staged_asset in generated_assets.iterdir():
            staged_asset.replace(final_assets / staged_asset.name)
    for mesh in model.findall("./asset/mesh"):
        path = Path(mesh.get("file", ""))
        if path.parent == generated_assets:
            mesh.set("file", str(final_assets / path.name))
    ET.indent(model, space="  ")
    final_payload = ET.tostring(model, encoding="utf-8", xml_declaration=True)
    with tempfile.NamedTemporaryFile(dir=output_path.parent, prefix="mickrobot-", suffix=".xml", delete=False) as temporary:
        temporary.write(final_payload)
        final_temporary_path = Path(temporary.name)
    final_temporary_path.replace(output_path)
    staging_context.cleanup()
    return output_path
