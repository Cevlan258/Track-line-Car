import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class LoraFixedModeTest(unittest.TestCase):
    def test_config_matches_team22_ewt22a_left_track_schedule(self):
        config = (PROJECT_ROOT / "App" / "Inc" / "app_config.h").read_text(encoding="utf-8")

        self.assertIn('#define APP_LORA_MODEL "EBYTE-EWT22A-900BWL22S"', config)
        self.assertIn('#define APP_TEAM_ID "22"', config)
        self.assertIn('#define APP_TEAM_NAME "CIRCUIT_VOYAGE"', config)
        self.assertIn('#define APP_LORA_LOCAL_ADDRESS 0x0001U', config)
        self.assertIn('#define APP_LORA_TARGET_ADDRESS 0x0001U', config)
        self.assertIn('#define APP_LORA_CHANNEL 10U', config)
        self.assertIn('#define APP_LORA_AIR_RATE 2U', config)
        self.assertIn('#define APP_LORA_NETWORK_ID 0U', config)
        self.assertIn('#define APP_LORA_PACKET_LENGTH 0U', config)
        self.assertIn('#define APP_LORA_FIXED_MODE 1U', config)
        self.assertIn('#define APP_LORA_RELAY_DISABLED 0U', config)
        self.assertIn('#define APP_LORA_KEY_DISABLED 0U', config)
        self.assertNotIn('APP_LORA_TRANSPARENT_MODE', config)

    def test_lora_init_configures_fixed_mode(self):
        source = (PROJECT_ROOT / "App" / "Src" / "lora.c").read_text(encoding="utf-8")

        self.assertIn('#define LORA_HMODE_CONFIG "AT+HMODE=0"', source)
        self.assertIn('#define LORA_HMODE_UART_LORA "AT+HMODE=1"', source)
        self.assertIn('lora_send_u32_command("AT+TRANS=", APP_LORA_FIXED_MODE)', source)
        self.assertIn('lora_send_u32_command("AT+ADDR=", APP_LORA_LOCAL_ADDRESS)', source)
        self.assertIn('lora_send_u32_command("AT+CHANNEL=", APP_LORA_CHANNEL)', source)
        self.assertNotIn("AT+MODE=", source)

    def test_checkpoint_packet_uses_fixed_mode_prefix(self):
        source = (PROJECT_ROOT / "App" / "Src" / "lora.c").read_text(encoding="utf-8")

        self.assertIn("LORA_FIXED_PREFIX_LEN", source)
        self.assertIn("packet[0] = (uint8_t)((APP_LORA_TARGET_ADDRESS >> 8U) & 0xFFU);", source)
        self.assertIn("packet[1] = (uint8_t)(APP_LORA_TARGET_ADDRESS & 0xFFU);", source)
        self.assertIn("packet[2] = (uint8_t)(APP_LORA_CHANNEL & 0xFFU);", source)
        self.assertIn("HAL_UART_Transmit(&huart2, packet", source)

    def test_docs_reference_ewt22a_not_old_lr22_model(self):
        context = (PROJECT_ROOT / "CONTEXT.md").read_text(encoding="utf-8")
        requirements = (PROJECT_ROOT / "需求.md").read_text(encoding="utf-8")
        combined = context + "\n" + requirements

        self.assertIn("EBYTE-EWT22A-900BWL22S", combined)
        self.assertIn("900MHz", requirements)
        self.assertNotIn("LR22-900T22D", combined)
        self.assertNotIn("868MHz", combined)


if __name__ == "__main__":
    unittest.main()
