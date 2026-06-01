import re
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DISPLAY_H = PROJECT_ROOT / "App" / "Inc" / "display_ssd1309.h"
DISPLAY_C = PROJECT_ROOT / "App" / "Src" / "display_ssd1309.c"
APP_STATE_C = PROJECT_ROOT / "App" / "Src" / "app_state.c"


class OledDisplayStaticTest(unittest.TestCase):
    def test_status_screen_accepts_and_renders_battery_percent(self):
        header = DISPLAY_H.read_text(encoding="utf-8")
        source = DISPLAY_C.read_text(encoding="utf-8")

        self.assertIn(
            "Display_ShowStatus(const char *state, uint32_t elapsed_ms, uint8_t battery_percent, const char *current_time)",
            header,
        )
        self.assertIn(
            "Display_ShowStatus(const char *state, uint32_t elapsed_ms, uint8_t battery_percent, const char *current_time)",
            source,
        )
        self.assertRegex(source, r"snprintf\(line, sizeof\(line\), \"%3u%%\", battery_percent\)")
        self.assertRegex(source, r"fb_text\(108,\s*0,\s*line\)")

    def test_status_screen_accepts_and_renders_current_time(self):
        header = DISPLAY_H.read_text(encoding="utf-8")
        source = DISPLAY_C.read_text(encoding="utf-8")

        self.assertIn("const char *current_time", header)
        self.assertIn("const char *current_time", source)
        self.assertRegex(source, r"snprintf\(line, sizeof\(line\), \"CLK %s\", current_time\)")
        self.assertRegex(source, r"fb_text\(0,\s*30,\s*line\)")

    def test_app_state_passes_latest_battery_percent_to_status_screen(self):
        source = APP_STATE_C.read_text(encoding="utf-8")

        self.assertIn("static uint8_t latest_battery_percent;", source)
        self.assertIn("latest_battery_percent = battery_percent;", source)
        self.assertRegex(
            source,
            r"Display_ShowStatus\(state_name\(app_state\), AppStart_ElapsedMs\(\), latest_battery_percent,\s*current_time\)",
        )
        self.assertRegex(
            source,
            r"Display_ShowStatus\(\"RTC ERR\", 0, latest_battery_percent,\s*\"--:--:--\"\)",
        )

    def test_app_state_reads_beijing_time_for_status_screen(self):
        source = APP_STATE_C.read_text(encoding="utf-8")

        self.assertIn("char current_time[9];", source)
        self.assertIn("AppTime_GetBeijingTime(current_time, sizeof(current_time));", source)


if __name__ == "__main__":
    unittest.main()
