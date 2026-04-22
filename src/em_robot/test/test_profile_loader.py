from pathlib import Path

from em_robot.profile_loader import load_profile, resolve_profile_path


def test_resolve_profile_path_prefers_explicit_path(tmp_path):
    profile_path = tmp_path / "custom.yaml"
    profile_path.write_text("profile_name: custom\n", encoding="utf-8")

    resolved_path = resolve_profile_path(profile_path=str(profile_path))

    assert resolved_path == str(profile_path)


def test_load_profile_reads_yaml_from_explicit_path(tmp_path):
    profile_path = tmp_path / "custom.yaml"
    profile_path.write_text("profile_name: test_profile\nmovement:\n  backend: fake\n", encoding="utf-8")

    profile, resolved_path = load_profile(profile_path=str(profile_path))

    assert profile["profile_name"] == "test_profile"
    assert profile["movement"]["backend"] == "fake"
    assert Path(resolved_path) == profile_path
