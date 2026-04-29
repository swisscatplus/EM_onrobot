from argparse import Namespace
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from em_robot.localization_capture import DEFAULT_TOPICS, build_capture_config, build_topics


def test_build_topics_keeps_defaults_and_deduplicates():
    topics = build_topics(
        extra_topics=["/odomWheel", "/extra/topic"],
        include_debug_image=False,
    )

    assert "/localization/debug_image" not in topics
    assert "/extra/topic" in topics
    assert topics.count("/odomWheel") == 1
    assert topics[0] == "/tf"


def test_build_capture_config_uses_timestamped_folder_name():
    args = Namespace(
        duration=12.5,
        output_dir="captures",
        name="test_run",
        extra_topics=[],
        no_debug_image=False,
        include_hidden_topics=True,
    )

    config = build_capture_config(args)

    assert config.duration_s == 12.5
    assert config.output_dir.parent == Path("captures").resolve()
    assert config.output_dir.name.startswith("test_run_")
    assert config.topics == DEFAULT_TOPICS
    assert config.include_hidden_topics is True
