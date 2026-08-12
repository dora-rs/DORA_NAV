"""Exact Dora wire contracts for simulated sensors."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import struct
from typing import Any, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray


POINTCLOUD_HEADER = struct.Struct("<dI")
IMAGE_HEADER = struct.Struct("<dIIB")
RGB8_ENCODING = 1
Vector3 = tuple[float, float, float]
QuaternionXyzw = tuple[float, float, float, float]


class MessageError(ValueError):
    """Raised when a sample cannot satisfy its wire contract."""


@dataclass(frozen=True)
class ImageSample:
    timestamp_s: float
    frame_id: str
    rgb: NDArray[np.uint8]


@dataclass(frozen=True)
class ImuSample:
    timestamp_s: float
    frame_id: str
    orientation_xyzw: QuaternionXyzw
    angular_velocity_rad_s: Vector3
    linear_acceleration_g: Vector3


@dataclass(frozen=True)
class OdometrySample:
    timestamp_s: float
    frame_id: str
    child_frame_id: str
    position_m: Vector3
    orientation_xyzw: QuaternionXyzw
    linear_velocity_mps: Vector3
    angular_velocity_rad_s: Vector3


@dataclass(frozen=True)
class GnssSample:
    timestamp_s: float
    frame_id: str
    fix: bool
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    velocity_enu_mps: Vector3
    position_covariance: tuple[float, float, float, float, float, float, float, float, float]


def _finite(values: Sequence[float], context: str) -> None:
    if not all(math.isfinite(float(value)) for value in values):
        raise MessageError(f"{context} values must be finite")


def encode_pointcloud(timestamp_s: float, points_xyzi: ArrayLike) -> bytes:
    _finite((timestamp_s,), "pointcloud")
    points = np.asarray(points_xyzi)
    if points.ndim != 2 or points.shape[1] != 4:
        raise MessageError(f"pointcloud must have shape (N, 4), got {points.shape}")
    if points.shape[0] > 0xFFFFFFFF:
        raise MessageError("pointcloud contains more than uint32 points")
    if not np.all(np.isfinite(points)):
        raise MessageError("pointcloud values must be finite")
    packed = np.ascontiguousarray(points, dtype="<f4")
    return POINTCLOUD_HEADER.pack(float(timestamp_s), len(packed)) + packed.tobytes()


def decode_pointcloud(payload: bytes) -> tuple[float, NDArray[np.float32]]:
    if len(payload) < POINTCLOUD_HEADER.size:
        raise MessageError("pointcloud payload length is shorter than its header")
    timestamp_s, count = POINTCLOUD_HEADER.unpack_from(payload)
    expected = POINTCLOUD_HEADER.size + count * 16
    if len(payload) != expected:
        raise MessageError(f"pointcloud payload length must be {expected}, got {len(payload)}")
    _finite((timestamp_s,), "pointcloud")
    points = np.frombuffer(payload, dtype="<f4", offset=POINTCLOUD_HEADER.size).reshape(count, 4).copy()
    if not np.all(np.isfinite(points)):
        raise MessageError("pointcloud values must be finite")
    return timestamp_s, points


def encode_image(sample: ImageSample) -> bytes:
    _finite((sample.timestamp_s,), "image")
    image = np.asarray(sample.rgb)
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise MessageError(f"RGB image must have uint8 shape (H, W, 3), got {image.dtype} {image.shape}")
    height, width, _ = image.shape
    if width <= 0 or height <= 0:
        raise MessageError("RGB image dimensions must be positive")
    return IMAGE_HEADER.pack(sample.timestamp_s, width, height, RGB8_ENCODING) + np.ascontiguousarray(image).tobytes()


def decode_image(payload: bytes, frame_id: str = "camera_link") -> ImageSample:
    if len(payload) < IMAGE_HEADER.size:
        raise MessageError("image payload length is shorter than its header")
    timestamp_s, width, height, encoding = IMAGE_HEADER.unpack_from(payload)
    if encoding != RGB8_ENCODING:
        raise MessageError(f"unsupported image encoding: {encoding}")
    expected = IMAGE_HEADER.size + width * height * 3
    if width == 0 or height == 0 or len(payload) != expected:
        raise MessageError(f"image payload length must be {expected}, got {len(payload)}")
    _finite((timestamp_s,), "image")
    rgb = np.frombuffer(payload, dtype=np.uint8, offset=IMAGE_HEADER.size).reshape(height, width, 3).copy()
    return ImageSample(timestamp_s, frame_id, rgb)


def _vector(values: Sequence[float], name: str, length: int) -> list[float]:
    if len(values) != length:
        raise MessageError(f"{name} must contain {length} values")
    _finite(values, name)
    return [float(value) for value in values]


def _vector3_dict(values: Vector3, name: str) -> dict[str, float]:
    vector = _vector(values, name, 3)
    return {"x": vector[0], "y": vector[1], "z": vector[2]}


def _quaternion_dict(values: QuaternionXyzw, name: str) -> dict[str, float]:
    quaternion = _vector(values, name, 4)
    return {"x": quaternion[0], "y": quaternion[1], "z": quaternion[2], "w": quaternion[3]}


def _json_bytes(value: dict[str, Any]) -> bytes:
    try:
        return json.dumps(value, separators=(",", ":"), allow_nan=False).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise MessageError(f"JSON message values must be finite and serializable: {exc}") from exc


def _json_object(payload: bytes) -> dict[str, Any]:
    def reject_constant(value: str) -> None:
        raise MessageError(f"JSON message contains non-finite value: {value}")

    try:
        result = json.loads(payload.decode("utf-8"), parse_constant=reject_constant)
    except MessageError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise MessageError(f"message is not valid UTF-8 JSON: {exc}") from exc
    if not isinstance(result, dict):
        raise MessageError("JSON message root must be an object")
    return result


def _object(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise MessageError(f"{name} must be an object")
    return value


def _decoded_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise MessageError(f"{name} must be a finite number")
    return float(value)


def _decoded_vector(value: Any, name: str) -> Vector3:
    obj = _object(value, name)
    return tuple(_decoded_number(obj.get(axis), f"{name}.{axis}") for axis in ("x", "y", "z"))  # type: ignore[return-value]


def _decoded_quaternion(value: Any, name: str) -> QuaternionXyzw:
    obj = _object(value, name)
    return tuple(_decoded_number(obj.get(axis), f"{name}.{axis}") for axis in ("x", "y", "z", "w"))  # type: ignore[return-value]


def _header(value: Any) -> tuple[float, str]:
    header = _object(value, "header")
    frame_id = header.get("frame_id")
    if not isinstance(frame_id, str):
        raise MessageError("header.frame_id must be a string")
    return _decoded_number(header.get("timestamp"), "header.timestamp"), frame_id


def encode_imu(sample: ImuSample) -> bytes:
    _finite((sample.timestamp_s,), "IMU")
    return _json_bytes({
        "header": {"timestamp": sample.timestamp_s, "frame_id": sample.frame_id},
        "orientation": _quaternion_dict(sample.orientation_xyzw, "IMU orientation"),
        "angular_velocity": _vector3_dict(sample.angular_velocity_rad_s, "IMU angular velocity"),
        "linear_acceleration": _vector3_dict(sample.linear_acceleration_g, "IMU acceleration"),
    })


def decode_imu(payload: bytes) -> ImuSample:
    value = _json_object(payload)
    timestamp, frame_id = _header(value.get("header"))
    return ImuSample(timestamp, frame_id, _decoded_quaternion(value.get("orientation"), "orientation"), _decoded_vector(value.get("angular_velocity"), "angular_velocity"), _decoded_vector(value.get("linear_acceleration"), "linear_acceleration"))


def encode_odometry(sample: OdometrySample) -> bytes:
    _finite((sample.timestamp_s,), "odometry")
    return _json_bytes({
        "header": {"timestamp": sample.timestamp_s, "frame_id": sample.frame_id},
        "child_frame_id": sample.child_frame_id,
        "pose": {"position": _vector3_dict(sample.position_m, "odometry position"), "orientation": _quaternion_dict(sample.orientation_xyzw, "odometry orientation")},
        "twist": {"linear": _vector3_dict(sample.linear_velocity_mps, "odometry linear velocity"), "angular": _vector3_dict(sample.angular_velocity_rad_s, "odometry angular velocity")},
    })


def decode_odometry(payload: bytes) -> OdometrySample:
    value = _json_object(payload)
    timestamp, frame_id = _header(value.get("header"))
    child = value.get("child_frame_id")
    if not isinstance(child, str):
        raise MessageError("child_frame_id must be a string")
    pose = _object(value.get("pose"), "pose")
    twist = _object(value.get("twist"), "twist")
    return OdometrySample(timestamp, frame_id, child, _decoded_vector(pose.get("position"), "pose.position"), _decoded_quaternion(pose.get("orientation"), "pose.orientation"), _decoded_vector(twist.get("linear"), "twist.linear"), _decoded_vector(twist.get("angular"), "twist.angular"))


def encode_gnss(sample: GnssSample) -> bytes:
    covariance = _vector(sample.position_covariance, "GNSS position covariance", 9)
    _finite((sample.timestamp_s, sample.latitude_deg, sample.longitude_deg, sample.altitude_m), "GNSS")
    return _json_bytes({
        "header": {"timestamp": sample.timestamp_s, "frame_id": sample.frame_id},
        "status": {"fix": bool(sample.fix), "service": "GPS"},
        "latitude": sample.latitude_deg, "longitude": sample.longitude_deg, "altitude": sample.altitude_m,
        "velocity_enu": {"east": sample.velocity_enu_mps[0], "north": sample.velocity_enu_mps[1], "up": sample.velocity_enu_mps[2]},
        "position_covariance": covariance,
    })


def decode_gnss(payload: bytes) -> GnssSample:
    value = _json_object(payload)
    timestamp, frame_id = _header(value.get("header"))
    status = _object(value.get("status"), "status")
    fix = status.get("fix")
    if not isinstance(fix, bool):
        raise MessageError("status.fix must be boolean")
    velocity = _object(value.get("velocity_enu"), "velocity_enu")
    covariance_value = value.get("position_covariance")
    if not isinstance(covariance_value, list) or len(covariance_value) != 9:
        raise MessageError("position_covariance must contain 9 values")
    covariance = tuple(_decoded_number(item, f"position_covariance[{index}]") for index, item in enumerate(covariance_value))
    return GnssSample(timestamp, frame_id, fix, _decoded_number(value.get("latitude"), "latitude"), _decoded_number(value.get("longitude"), "longitude"), _decoded_number(value.get("altitude"), "altitude"), (_decoded_number(velocity.get("east"), "velocity_enu.east"), _decoded_number(velocity.get("north"), "velocity_enu.north"), _decoded_number(velocity.get("up"), "velocity_enu.up")), covariance)  # type: ignore[arg-type]
