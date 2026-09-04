from io import BytesIO
from pathlib import Path
import zipfile

import pytest
import yaml

from studica_robot_platform.map_registry import MapRegistry, MapRegistryError


def bundle(
    yaml_name="office.yaml",
    image_name="office.pgm",
    image=b"P5\n1 1\n255\n\xff",
    **overrides,
) -> bytes:
    document = {
        "image": image_name,
        "resolution": 0.05,
        "origin": [-1.0, -2.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
        "mode": "trinary",
    }
    document.update(overrides)
    output = BytesIO()
    with zipfile.ZipFile(output, "w") as archive:
        archive.writestr(yaml_name, yaml.safe_dump(document))
        archive.writestr(image_name, image)
    return output.getvalue()


def test_registry_normalizes_and_round_trips(tmp_path: Path):
    registry = MapRegistry(tmp_path / "maps")
    metadata = registry.import_bundle("office_nav", bundle())
    assert metadata["map_id"] == "office_nav"
    exported = registry.export_bundle("office_nav")
    with zipfile.ZipFile(BytesIO(exported)) as archive:
        assert sorted(archive.namelist()) == ["map.pgm", "map.yaml"]
    second = MapRegistry(tmp_path / "second")
    second.import_bundle("copy", exported)
    assert second.yaml_path("copy").is_file()


def test_registry_rejects_traversal_and_mutation(tmp_path: Path):
    output = BytesIO()
    with zipfile.ZipFile(output, "w") as archive:
        archive.writestr("../map.yaml", "image: map.pgm")
        archive.writestr("map.pgm", b"x")
    registry = MapRegistry(tmp_path / "maps")
    with pytest.raises(MapRegistryError):
        registry.import_bundle("unsafe", output.getvalue())
    registry.import_bundle("fixed", bundle())
    with pytest.raises(MapRegistryError):
        registry.import_bundle("fixed", bundle())


@pytest.mark.parametrize(
    "payload",
    (
        bundle(resolution=float("nan")),
        bundle(origin=[0.0, float("inf"), 0.0]),
        bundle(image=b"P5\n1 2\n255\n\xff"),
        bundle(image=b"not-a-pgm"),
    ),
)
def test_registry_rejects_non_finite_metadata_and_bad_images(
    tmp_path: Path, payload: bytes
):
    registry = MapRegistry(tmp_path / "maps")
    with pytest.raises(MapRegistryError):
        registry.import_bundle("invalid", payload)
