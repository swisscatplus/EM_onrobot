from __future__ import annotations


def clamp_unit(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def scale_color(color: tuple[float, float, float], brightness: float = 1.0) -> tuple[float, float, float]:
    scale = clamp_unit(brightness)
    return tuple(clamp_unit(channel) * scale for channel in color)


def resolve_color_pins(pins: list[int] | tuple[int, int, int], color_order: str) -> tuple[int, int, int]:
    if len(pins) != 3:
        raise ValueError(f"RGB LEDs require exactly 3 GPIO pins, got {len(pins)}")

    normalized_order = color_order.strip().lower()
    if sorted(normalized_order) != ["b", "g", "r"]:
        raise ValueError(
            "color_order must be a permutation of 'rgb', "
            f"got '{color_order}'"
        )

    pin_lookup = dict(zip(normalized_order, [int(pin) for pin in pins]))
    return pin_lookup["r"], pin_lookup["g"], pin_lookup["b"]
