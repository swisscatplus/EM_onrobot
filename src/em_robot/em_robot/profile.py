#!/usr/bin/env python3
import os

import yaml


DEFAULT_PROFILE = "real_robot"


def get_profiles_directory():
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory("em_robot"), "config", "profiles")


def resolve_profile_path(profile_name=None, profile_path=None):
    if profile_path:
        return profile_path

    selected_profile = profile_name or os.environ.get("EM_ROBOT_PROFILE", DEFAULT_PROFILE)
    return os.path.join(get_profiles_directory(), f"{selected_profile}.yaml")


def load_profile(profile_name=None, profile_path=None):
    resolved_path = resolve_profile_path(profile_name=profile_name, profile_path=profile_path)
    with open(resolved_path, "r", encoding="utf-8") as profile_file:
        return yaml.safe_load(profile_file), resolved_path
