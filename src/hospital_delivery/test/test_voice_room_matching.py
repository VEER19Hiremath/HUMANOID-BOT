# Copyright 2026 HUMANOID-BOT
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Unit tests for voice room / navigation phrase matching."""

import pytest

from hospital_delivery.voice_delivery_node import VoiceDeliveryNode


@pytest.fixture
def voice():
    return VoiceDeliveryNode.__new__(VoiceDeliveryNode)


@pytest.mark.parametrize(
    "heard,expected",
    [
        ("go to room one", "room1"),
        ("go to rome one", "room1"),
        ("gotta one", "room1"),
        ("go to the one", "room1"),
        ("go to room 2", "room2"),
        ("go to toronto", "room2"),
        ("go to room three", "room3"),
        ("go home", "home"),
        ("go to reception", "home"),
        ("go to start", "home"),
    ],
)
def test_extract_room_positives(voice, heard, expected):
    assert voice.extract_room(heard) == expected


@pytest.mark.parametrize(
    "heard",
    [
        "now",
        "shh",
        "forgot my keys",
        "got around to it",
        "toronto maple",
        "please start the meeting",
        "run away",
        "from here",
    ],
)
def test_extract_room_negatives(voice, heard):
    assert voice.extract_room(heard) is None


@pytest.mark.parametrize(
    "heard,expected",
    [
        ("go to room one", True),
        ("gotta one", True),
        ("go home", True),
        ("room two please", True),
        ("hello there", False),
        ("forgot", False),
        ("toronto maple", False),
        ("please start the meeting", False),
    ],
)
def test_is_navigation_command(voice, heard, expected):
    assert voice.is_navigation_command(heard) is expected


def test_stop_robot_uses_goal_handle_cancel():
    """Cancel path must use goal-handle API, not ActionClient.cancel_all_goals_async."""
    import inspect

    src = inspect.getsource(VoiceDeliveryNode.stop_robot)
    assert "cancel_all_goals_async" not in src
    assert "cancel_goal_async" in src
