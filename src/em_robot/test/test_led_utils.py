from em_robot.led_utils import resolve_color_pins, scale_color


def test_resolve_color_pins_respects_color_order():
    red_pin, green_pin, blue_pin = resolve_color_pins((23, 24, 25), "grb")

    assert red_pin == 24
    assert green_pin == 23
    assert blue_pin == 25


def test_scale_color_clamps_channels_and_brightness():
    scaled = scale_color((1.2, 0.5, -0.1), brightness=0.5)

    assert scaled == (0.5, 0.25, 0.0)
