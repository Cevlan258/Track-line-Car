import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def read_source(relative_path):
    return (PROJECT_ROOT / relative_path).read_text(encoding="utf-8")


class Stm32InnerWheelSlowdownTest(unittest.TestCase):
    def test_turn_direction_and_slowdown_macros_are_configurable(self):
        config = read_source("App/Inc/app_config.h")

        self.assertIn("APP_TURN_POSITIVE_YAW_SLOWS_MOTOR_A", config)
        self.assertIn("#define APP_TURN_INNER_MIN_SCALE_PCT 35", config)
        self.assertIn("APP_TURN_INNER_SLOWDOWN_PCT", config)

    def test_kinematics_mixes_yaw_into_inner_wheel_only(self):
        source = read_source("App/Src/ax_kinematics.c")

        self.assertIn('#include "app_config.h"', source)
        self.assertIn("turn_ratio", source)
        self.assertIn("inner_scale", source)
        self.assertIn("APP_TURN_POSITIVE_YAW_SLOWS_MOTOR_A", source)
        self.assertIn("R_Vel.TG_IW > 0", source)
        self.assertIn("R_Wheel_A.TG *= inner_scale", source)
        self.assertIn("R_Wheel_B.TG *= inner_scale", source)
        self.assertIn("apply_inner_wheel_slowdown();", source)


if __name__ == "__main__":
    unittest.main()
