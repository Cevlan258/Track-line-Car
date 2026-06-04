import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
APP_STATE_C = PROJECT_ROOT / "App" / "Src" / "app_state.c"
DISPLAY_C = PROJECT_ROOT / "App" / "Src" / "display_ssd1309.c"
DISPLAY_H = PROJECT_ROOT / "App" / "Inc" / "display_ssd1309.h"


class OledRadarDisplayTest(unittest.TestCase):
    def test_avoiding_command_selects_oled_radar_scope(self):
        source = APP_STATE_C.read_text(encoding="utf-8")

        self.assertIn("MAIX_LINK_CMD_FLAG_AVOIDING", source)
        self.assertIn("Display_ShowRadarScope", source)
        self.assertIn("last_command_flags", source)

    def test_oled_radar_scope_draws_fan_grid_from_radar_targets(self):
        display_source = DISPLAY_C.read_text(encoding="utf-8")
        header_source = DISPLAY_H.read_text(encoding="utf-8")

        self.assertIn("void Display_ShowRadarScope(const RadarSample *sample, uint8_t avoid_flags", header_source)
        self.assertIn("Display_ShowRadarScope", display_source)
        self.assertIn("RADAR_SCOPE_RINGS", display_source)
        self.assertIn("draw_radar_arc", display_source)
        self.assertIn("sample->targets", display_source)

    def test_status_and_radar_pages_receive_current_time_text(self):
        app_state_source = APP_STATE_C.read_text(encoding="utf-8")
        display_source = DISPLAY_C.read_text(encoding="utf-8")
        header_source = DISPLAY_H.read_text(encoding="utf-8")

        self.assertIn("AppTime_GetBeijingTime(time_text", app_state_source)
        self.assertIn("Display_ShowStatusTime(display_state_name(), AppStart_ElapsedMs(), time_text)", app_state_source)
        self.assertIn("Display_ShowRadarScope(&sample, last_command_flags, time_text)", app_state_source)
        self.assertIn("void Display_ShowStatusTime(const char *state, uint32_t elapsed_ms, const char *time_text)", header_source)
        self.assertIn("time_text", display_source)


if __name__ == "__main__":
    unittest.main()
