"""Validated, traversal-safe Nav2 map registry."""

from __future__ import annotations

from io import BytesIO
import json
import math
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import struct
import tempfile
from typing import Any, Dict, List
import zipfile
import zlib

import yaml

from .model import valid_map_id


MAX_MAP_BUNDLE_BYTES = 32 * 1024 * 1024
MAX_MAP_YAML_BYTES = 64 * 1024
IMAGE_SUFFIXES = {".pgm", ".png"}
MAX_MAP_DIMENSION = 20_000
MAX_MAP_PIXELS = 25_000_000


class MapRegistryError(ValueError):
    """A map cannot be accepted without violating registry policy."""


def _safe_zip_members(archive: zipfile.ZipFile) -> List[zipfile.ZipInfo]:
    members = archive.infolist()
    if not 1 <= len(members) <= 4:
        raise MapRegistryError("map bundle must contain a YAML and one image")
    total = 0
    files: List[zipfile.ZipInfo] = []
    for member in members:
        path = PurePosixPath(member.filename)
        if path.is_absolute() or ".." in path.parts or len(path.parts) != 1:
            raise MapRegistryError("map bundle contains an unsafe path")
        mode = member.external_attr >> 16
        if stat.S_ISLNK(mode):
            raise MapRegistryError("map bundle cannot contain symlinks")
        if member.is_dir():
            continue
        total += member.file_size
        if member.file_size < 0 or total > MAX_MAP_BUNDLE_BYTES:
            raise MapRegistryError("map bundle is too large")
        files.append(member)
    return files


def validate_map_yaml(document: Any, image_name: str) -> Dict[str, Any]:
    """Validate the portable subset required by Nav2 map_server."""
    if not isinstance(document, dict):
        raise MapRegistryError("map YAML must contain an object")
    required = {
        "image",
        "resolution",
        "origin",
        "negate",
        "occupied_thresh",
        "free_thresh",
        "mode",
    }
    missing = sorted(required.difference(document))
    if missing:
        raise MapRegistryError(f"map YAML is missing: {', '.join(missing)}")
    unknown = sorted(set(document).difference(required))
    if unknown:
        raise MapRegistryError(f"map YAML has unsupported fields: {', '.join(unknown)}")
    if not isinstance(document["image"], str):
        raise MapRegistryError("map YAML image must be a string")
    if Path(str(document["image"])).name != image_name:
        raise MapRegistryError("map YAML image must name the bundled image")
    if Path(str(document["image"])).is_absolute() or ".." in Path(
        str(document["image"])
    ).parts:
        raise MapRegistryError("map YAML image path must be relative")
    if not isinstance(document["origin"], (list, tuple)) or len(
        document["origin"]
    ) != 3:
        raise MapRegistryError("map origin must contain x, y, yaw")
    try:
        resolution = float(document["resolution"])
        occupied = float(document["occupied_thresh"])
        free = float(document["free_thresh"])
        origin = [float(value) for value in document["origin"]]
        negate = int(document["negate"])
    except (TypeError, ValueError) as error:
        raise MapRegistryError("map YAML contains invalid numeric fields") from error
    if not all(math.isfinite(value) for value in (resolution, occupied, free, *origin)):
        raise MapRegistryError("map numeric fields must be finite")
    if resolution <= 0.0 or resolution > 1.0:
        raise MapRegistryError("map resolution must be in (0, 1]")
    if abs(origin[0]) > 100_000 or abs(origin[1]) > 100_000 or abs(origin[2]) > 1000:
        raise MapRegistryError("map origin is outside accepted bounds")
    if not 0.0 <= free < occupied <= 1.0:
        raise MapRegistryError("map thresholds must satisfy 0 <= free < occupied <= 1")
    if negate not in {0, 1}:
        raise MapRegistryError("map negate must be 0 or 1")
    mode = str(document["mode"]).lower()
    if mode not in {"trinary", "scale", "raw"}:
        raise MapRegistryError("map mode is unsupported")
    return {
        "image": image_name,
        "resolution": resolution,
        "origin": origin,
        "negate": negate,
        "occupied_thresh": occupied,
        "free_thresh": free,
        "mode": mode,
    }


def _validate_dimensions(width: int, height: int) -> None:
    if not 1 <= width <= MAX_MAP_DIMENSION or not 1 <= height <= MAX_MAP_DIMENSION:
        raise MapRegistryError("map image dimensions are invalid")
    if width * height > MAX_MAP_PIXELS:
        raise MapRegistryError("map image contains too many pixels")


def _validate_png(payload: bytes) -> None:
    if not payload.startswith(b"\x89PNG\r\n\x1a\n"):
        raise MapRegistryError("map PNG signature is invalid")
    position = 8
    saw_header = False
    saw_data = False
    saw_end = False
    while position + 12 <= len(payload):
        length = struct.unpack(">I", payload[position:position + 4])[0]
        chunk_end = position + 12 + length
        if chunk_end > len(payload):
            raise MapRegistryError("map PNG is truncated")
        chunk_type = payload[position + 4:position + 8]
        chunk_data = payload[position + 8:position + 8 + length]
        expected_crc = struct.unpack(">I", payload[position + 8 + length:chunk_end])[0]
        if zlib.crc32(chunk_type + chunk_data) & 0xFFFFFFFF != expected_crc:
            raise MapRegistryError("map PNG checksum is invalid")
        if not saw_header:
            if chunk_type != b"IHDR" or length != 13:
                raise MapRegistryError("map PNG header is invalid")
            width, height = struct.unpack(">II", chunk_data[:8])
            _validate_dimensions(width, height)
            saw_header = True
        elif chunk_type == b"IDAT":
            saw_data = True
        elif chunk_type == b"IEND":
            if length != 0:
                raise MapRegistryError("map PNG end marker is invalid")
            saw_end = True
            position = chunk_end
            break
        position = chunk_end
    if not saw_header or not saw_data or not saw_end or position != len(payload):
        raise MapRegistryError("map PNG structure is invalid")


def _pgm_token(payload: bytes, position: int) -> tuple[bytes, int]:
    while position < len(payload):
        if payload[position] in b" \t\r\n":
            position += 1
            continue
        if payload[position] == ord("#"):
            newline = payload.find(b"\n", position)
            if newline < 0:
                return b"", len(payload)
            position = newline + 1
            continue
        break
    start = position
    while position < len(payload) and payload[position] not in b" \t\r\n#":
        position += 1
    return payload[start:position], position


def _pgm_integer(token: bytes, field: str) -> int:
    try:
        value = int(token.decode("ascii"))
    except (UnicodeDecodeError, ValueError) as error:
        raise MapRegistryError(f"map PGM {field} is invalid") from error
    return value


def _validate_pgm(payload: bytes) -> None:
    position = 0
    header = []
    for field in ("format", "width", "height", "maximum"):
        token, position = _pgm_token(payload, position)
        if not token:
            raise MapRegistryError(f"map PGM {field} is missing")
        header.append(token)
    magic = header[0]
    if magic not in {b"P2", b"P5"}:
        raise MapRegistryError("map PGM format must be P2 or P5")
    width = _pgm_integer(header[1], "width")
    height = _pgm_integer(header[2], "height")
    maximum = _pgm_integer(header[3], "maximum")
    _validate_dimensions(width, height)
    if not 1 <= maximum <= 65_535:
        raise MapRegistryError("map PGM maximum is invalid")
    pixel_count = width * height
    if magic == b"P5":
        if position >= len(payload) or payload[position] not in b" \t\r\n":
            raise MapRegistryError("map PGM raster separator is missing")
        position += 1
        if payload[position - 1] == ord("\r") and position < len(payload):
            if payload[position] == ord("\n"):
                position += 1
        bytes_per_pixel = 1 if maximum < 256 else 2
        if len(payload) - position != pixel_count * bytes_per_pixel:
            raise MapRegistryError("map PGM raster length is invalid")
        return
    for _ in range(pixel_count):
        token, position = _pgm_token(payload, position)
        value = _pgm_integer(token, "pixel")
        if not 0 <= value <= maximum:
            raise MapRegistryError("map PGM pixel is outside its declared range")
    token, _ = _pgm_token(payload, position)
    if token:
        raise MapRegistryError("map PGM has extra pixel data")


def validate_map_image(payload: bytes, suffix: str) -> None:
    """Validate the image structure before it can reach map_server."""
    if suffix == ".png":
        _validate_png(payload)
    elif suffix == ".pgm":
        _validate_pgm(payload)
    else:
        raise MapRegistryError("map image type is unsupported")


class MapRegistry:
    """Store immutable validated maps under one canonical root."""

    def __init__(self, root: Path) -> None:
        if not root.is_absolute():
            raise MapRegistryError("map root must be absolute")
        root.mkdir(parents=True, exist_ok=True)
        if root.is_symlink() or not root.is_dir():
            raise MapRegistryError("map root must be a real directory")
        self.root = root.resolve()

    def list_maps(self) -> List[Dict[str, Any]]:
        """Return metadata for every complete registry entry."""
        result = []
        for entry in sorted(self.root.iterdir()):
            metadata = entry / "metadata.json"
            if entry.is_dir() and not entry.is_symlink() and metadata.is_file():
                try:
                    value = json.loads(metadata.read_text(encoding="utf-8"))
                except (OSError, json.JSONDecodeError):
                    continue
                if isinstance(value, dict):
                    result.append(value)
        return result

    def yaml_path(self, map_id: str) -> Path:
        """Return the validated YAML path for an existing map."""
        self._validate_id(map_id)
        path = self.root / map_id / "map.yaml"
        if not path.is_file() or path.is_symlink():
            raise MapRegistryError("map does not exist")
        return path

    def import_bundle(self, map_id: str, payload: bytes) -> Dict[str, Any]:
        """Validate and atomically add one immutable ZIP map bundle."""
        self._validate_id(map_id)
        if len(payload) > MAX_MAP_BUNDLE_BYTES:
            raise MapRegistryError("map bundle is too large")
        destination = self.root / map_id
        if destination.exists():
            raise MapRegistryError("map_id already exists")
        try:
            archive = zipfile.ZipFile(BytesIO(payload))
        except zipfile.BadZipFile as error:
            raise MapRegistryError("map bundle is not a ZIP archive") from error
        with archive:
            members = _safe_zip_members(archive)
            yaml_members = [
                item
                for item in members
                if Path(item.filename).suffix in {".yaml", ".yml"}
            ]
            image_members = [
                item
                for item in members
                if Path(item.filename).suffix.lower() in IMAGE_SUFFIXES
            ]
            if len(yaml_members) != 1 or len(image_members) != 1 or len(members) != 2:
                raise MapRegistryError("map bundle must contain exactly one YAML and one image")
            yaml_bytes = archive.read(yaml_members[0])
            image_bytes = archive.read(image_members[0])
        if len(yaml_bytes) > MAX_MAP_YAML_BYTES:
            raise MapRegistryError("map YAML is too large")
        if not image_bytes:
            raise MapRegistryError("map image is empty")
        image_suffix = Path(image_members[0].filename).suffix.lower()
        validate_map_image(image_bytes, image_suffix)
        image_name = f"map{image_suffix}"
        try:
            document = yaml.safe_load(yaml_bytes.decode("utf-8"))
        except (UnicodeDecodeError, yaml.YAMLError) as error:
            raise MapRegistryError("map YAML cannot be parsed") from error
        source_image_name = Path(image_members[0].filename).name
        normalized = validate_map_yaml(document, source_image_name)
        normalized["image"] = image_name
        metadata = {
            "schema_version": 1,
            "map_id": map_id,
            "resolution": normalized["resolution"],
            "origin": normalized["origin"],
            "image": image_name,
        }
        staging = Path(tempfile.mkdtemp(prefix=f".{map_id}.", dir=self.root))
        try:
            (staging / "map.yaml").write_text(
                yaml.safe_dump(normalized, sort_keys=False), encoding="utf-8"
            )
            (staging / image_name).write_bytes(image_bytes)
            (staging / "metadata.json").write_text(
                json.dumps(metadata, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            os.replace(staging, destination)
        except Exception:
            shutil.rmtree(staging, ignore_errors=True)
            raise
        return metadata

    def export_bundle(self, map_id: str) -> bytes:
        """Create a bounded ZIP containing the normalized map pair."""
        yaml_path = self.yaml_path(map_id)
        document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        image_name = str(document["image"])
        image_path = yaml_path.parent / image_name
        if not image_path.is_file() or image_path.is_symlink():
            raise MapRegistryError("map image is missing")
        output = BytesIO()
        with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as archive:
            archive.write(yaml_path, "map.yaml")
            archive.write(image_path, image_name)
        payload = output.getvalue()
        if len(payload) > MAX_MAP_BUNDLE_BYTES:
            raise MapRegistryError("exported map bundle is too large")
        return payload

    @staticmethod
    def _validate_id(map_id: str) -> None:
        if not valid_map_id(map_id):
            raise MapRegistryError("invalid map_id")
