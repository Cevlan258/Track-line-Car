import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def read_source(relative_path):
    return (PROJECT_ROOT / relative_path).read_text(encoding="utf-8")


class Stm32FaultDiagnosticsTest(unittest.TestCase):
    def test_usart3_error_callback_restarts_maix_link_receive(self):
        source = read_source("App/Src/app_uart.c")

        self.assertIn("HAL_UART_ErrorCallback", source)
        self.assertIn("MaixLink_UartErrorCallback", source)

    def test_fault_display_distinguishes_link_timeout_and_command_fault(self):
        source = read_source("App/Src/app_state.c")

        self.assertIn('"F LINK"', source)
        self.assertIn('"F CMD"', source)


if __name__ == "__main__":
    unittest.main()
