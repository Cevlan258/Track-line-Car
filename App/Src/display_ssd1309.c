#include "display_ssd1309.h"
#include "app_start.h"
#include "main.h"
#include "spi.h"
#include <stdio.h>
#include <string.h>

#define OLED_W 128U
#define OLED_H 64U
#define OLED_PAGES 8U

static uint8_t fb[OLED_W * OLED_PAGES];
static uint8_t display_ready;

static void oled_select(void)
{
  HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);
}

static void oled_unselect(void)
{
  HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
}

static void oled_cmd(uint8_t cmd)
{
  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);
  oled_select();
  (void)HAL_SPI_Transmit(&hspi1, &cmd, 1, 20U);
  oled_unselect();
}

static void oled_data(const uint8_t *data, uint16_t len)
{
  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
  oled_select();
  (void)HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, 100U);
  oled_unselect();
}

static void fb_clear(void)
{
  memset(fb, 0, sizeof(fb));
}

static void fb_pixel(uint8_t x, uint8_t y, uint8_t on)
{
  if ((x >= OLED_W) || (y >= OLED_H))
  {
    return;
  }

  if (on != 0U)
  {
    fb[x + (y / 8U) * OLED_W] |= (uint8_t)(1U << (y & 7U));
  }
  else
  {
    fb[x + (y / 8U) * OLED_W] &= (uint8_t)~(1U << (y & 7U));
  }
}

static void fb_hline(uint8_t x0, uint8_t x1, uint8_t y)
{
  uint8_t x;
  for (x = x0; x <= x1 && x < OLED_W; x++)
  {
    fb_pixel(x, y, 1);
  }
}

static void fb_vline(uint8_t x, uint8_t y0, uint8_t y1)
{
  uint8_t y;
  for (y = y0; y <= y1 && y < OLED_H; y++)
  {
    fb_pixel(x, y, 1);
  }
}

static uint8_t glyph3x5(char c, uint8_t row)
{
  const char *rows = NULL;

  switch (c)
  {
    case '0': rows = "111101101101111"; break;
    case '1': rows = "010110010010111"; break;
    case '2': rows = "111001111100111"; break;
    case '3': rows = "111001111001111"; break;
    case '4': rows = "101101111001001"; break;
    case '5': rows = "111100111001111"; break;
    case '6': rows = "111100111101111"; break;
    case '7': rows = "111001010010010"; break;
    case '8': rows = "111101111101111"; break;
    case '9': rows = "111101111001111"; break;
    case 'A': rows = "010101111101101"; break;
    case 'B': rows = "110101110101110"; break;
    case 'C': rows = "111100100100111"; break;
    case 'D': rows = "110101101101110"; break;
    case 'E': rows = "111100110100111"; break;
    case 'F': rows = "111100110100100"; break;
    case 'G': rows = "111100101101111"; break;
    case 'H': rows = "101101111101101"; break;
    case 'I': rows = "111010010010111"; break;
    case 'J': rows = "001001001101111"; break;
    case 'K': rows = "101101110101101"; break;
    case 'L': rows = "100100100100111"; break;
    case 'M': rows = "101111111101101"; break;
    case 'N': rows = "101111111111101"; break;
    case 'O': rows = "111101101101111"; break;
    case 'P': rows = "111101111100100"; break;
    case 'Q': rows = "111101101111001"; break;
    case 'R': rows = "110101110101101"; break;
    case 'S': rows = "111100111001111"; break;
    case 'T': rows = "111010010010010"; break;
    case 'U': rows = "101101101101111"; break;
    case 'V': rows = "101101101101010"; break;
    case 'W': rows = "101101111111101"; break;
    case 'X': rows = "101101010101101"; break;
    case 'Y': rows = "101101010010010"; break;
    case 'Z': rows = "111001010100111"; break;
    case ':': rows = "000010000010000"; break;
    case '.': rows = "000000000000010"; break;
    case '%': rows = "101001010100101"; break;
    case '-': rows = "000000111000000"; break;
    case '?': rows = "111001010000010"; break;
    case '=': rows = "000111000111000"; break;
    case ' ': rows = "000000000000000"; break;
    default: rows = "000000000000000"; break;
  }

  return (uint8_t)(((rows[row * 3U] - '0') << 2U) |
                   ((rows[row * 3U + 1U] - '0') << 1U) |
                   (rows[row * 3U + 2U] - '0'));
}

static void fb_char(uint8_t x, uint8_t y, char c)
{
  uint8_t row;
  uint8_t col;

  if ((c >= 'a') && (c <= 'z'))
  {
    c = (char)(c - 'a' + 'A');
  }

  for (row = 0; row < 5U; row++)
  {
    const uint8_t bits = glyph3x5(c, row);
    for (col = 0; col < 3U; col++)
    {
      if ((bits & (uint8_t)(1U << (2U - col))) != 0U)
      {
        fb_pixel((uint8_t)(x + col), (uint8_t)(y + row), 1);
      }
    }
  }
}

static void fb_text(uint8_t x, uint8_t y, const char *s)
{
  while ((*s != '\0') && (x < (OLED_W - 4U)))
  {
    fb_char(x, y, *s++);
    x = (uint8_t)(x + 4U);
  }
}

static void fb_flush(void)
{
  uint8_t page;

  if (display_ready == 0U)
  {
    return;
  }

  for (page = 0; page < OLED_PAGES; page++)
  {
    oled_cmd((uint8_t)(0xB0U + page));
    oled_cmd(0x00);
    oled_cmd(0x10);
    oled_data(&fb[page * OLED_W], OLED_W);
  }
}

static char road_code(uint8_t road_type)
{
  switch ((AX_VisionRoadType)road_type)
  {
    case AX_VISION_ROAD_STRAIGHT: return 'S';
    case AX_VISION_ROAD_LEFT_CURVE: return 'L';
    case AX_VISION_ROAD_RIGHT_CURVE: return 'R';
    case AX_VISION_ROAD_FORK: return 'F';
    case AX_VISION_ROAD_CROSS: return 'X';
    case AX_VISION_ROAD_T_LEFT: return 'A';
    case AX_VISION_ROAD_T_RIGHT: return 'B';
    case AX_VISION_ROAD_FINISH: return 'E';
    case AX_VISION_ROAD_UNKNOWN:
    default: return '?';
  }
}

void Display_Init(void)
{
  HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET);
  HAL_Delay(20);

  oled_cmd(0xAE);
  oled_cmd(0xD5); oled_cmd(0x80);
  oled_cmd(0xA8); oled_cmd(0x3F);
  oled_cmd(0xD3); oled_cmd(0x00);
  oled_cmd(0x40);
  oled_cmd(0xA1);
  oled_cmd(0xC8);
  oled_cmd(0xDA); oled_cmd(0x12);
  oled_cmd(0x81); oled_cmd(0x7F);
  oled_cmd(0xA4);
  oled_cmd(0xA6);
  oled_cmd(0xD9); oled_cmd(0xF1);
  oled_cmd(0xDB); oled_cmd(0x40);
  oled_cmd(0x20); oled_cmd(0x02);
  oled_cmd(0x8D); oled_cmd(0x14);
  oled_cmd(0xAF);

  display_ready = 1;
  fb_clear();
  fb_flush();
}

void Display_ShowStatus(const char *state, uint32_t elapsed_ms, uint8_t battery_percent, const char *current_time)
{
  char line[18];
  const uint32_t elapsed_s = elapsed_ms / 1000U;

  fb_clear();
  fb_text(0, 0, "TRACK CAR");
  (void)snprintf(line, sizeof(line), "%3u%%", battery_percent);
  fb_text(108, 0, line);
  fb_text(0, 10, state);
  (void)snprintf(line, sizeof(line), "T%02lu:%02lu", elapsed_s / 60U, elapsed_s % 60U);
  fb_text(0, 20, line);
  (void)snprintf(line, sizeof(line), "CLK %s", current_time);
  fb_text(0, 30, line);
  fb_flush();
}

void Display_ShowLineSensor(const uint16_t *frame, const AX_CCD_LineInfo *info, const char *state, uint16_t vin_x100, uint8_t battery_percent)
{
  char line[18];

  fb_clear();
  fb_text(0, 0, state);
  (void)snprintf(line, sizeof(line), "%3u%%", battery_percent);
  fb_text(108, 0, line);

  if ((frame != NULL) && (info != NULL))
  {
    fb_hline(10, 117, 42);
    fb_vline(64, 22, 62);

    if (info->line_valid != 0U)
    {
      const uint8_t line_center = (uint8_t)(64 + info->offset);
      const uint8_t marker_l4 = (line_center > 4U) ? (uint8_t)(line_center - 4U) : 0U;
      const uint8_t marker_l3 = (line_center > 3U) ? (uint8_t)(line_center - 3U) : 0U;
      const uint8_t marker_l2 = (line_center > 2U) ? (uint8_t)(line_center - 2U) : 0U;
      const uint8_t marker_l1 = (line_center > 1U) ? (uint8_t)(line_center - 1U) : 0U;
      const uint8_t marker_r4 = (line_center < 123U) ? (uint8_t)(line_center + 4U) : 127U;
      const uint8_t marker_r3 = (line_center < 124U) ? (uint8_t)(line_center + 3U) : 127U;
      const uint8_t marker_r2 = (line_center < 125U) ? (uint8_t)(line_center + 2U) : 127U;
      const uint8_t marker_r1 = (line_center < 126U) ? (uint8_t)(line_center + 1U) : 127U;
      fb_hline(marker_l4, marker_r4, 24);
      fb_hline(marker_l3, marker_r3, 25);
      fb_hline(marker_l2, marker_r2, 26);
      fb_hline(marker_l1, marker_r1, 27);
      fb_pixel(line_center, 28, 1);
      fb_vline(line_center, 30, 62);
      fb_hline(info->left_edge, info->right_edge, 56);
      fb_hline(info->left_edge, info->right_edge, 57);
    }
    else
    {
      fb_text(48, 28, "NO LINE");
    }

    (void)snprintf(line, sizeof(line), "MV%c O%03d %c%03u",
                   (info->fresh != 0U) ? 'K' : '-',
                   info->offset,
                   road_code(info->road_type),
                   info->confidence);
    fb_text(0, 8, line);
    (void)snprintf(line, sizeof(line), "%02u.%01uV", vin_x100 / 100U, (vin_x100 % 100U) / 10U);
    fb_text(104, 8, line);
  }

  fb_flush();
}

void Display_ShowLineGate(const uint16_t *frame, const AX_CCD_LineInfo *info, const RadarSample *sample, const char *state, uint8_t gate_count, const char *gate_state, uint16_t gate_score, uint16_t vin_x100, uint8_t battery_percent)
{
  char line[18];
  uint16_t radar_distance = 0;

  fb_clear();
  fb_text(0, 0, state);
  (void)snprintf(line, sizeof(line), "%3u%%", battery_percent);
  fb_text(108, 0, line);

  if ((sample != NULL) && (sample->valid != 0U))
  {
    radar_distance = sample->detect_distance_cm;
  }
  (void)snprintf(line, sizeof(line), "D%03u E%03u", radar_distance, gate_score);
  fb_text(0, 8, line);
  (void)snprintf(line, sizeof(line), "G%u %s", gate_count, gate_state);
  fb_text(76, 8, line);

  if ((frame != NULL) && (info != NULL))
  {
    fb_hline(10, 117, 42);
    fb_vline(64, 22, 62);

    if (info->line_valid != 0U)
    {
      const uint8_t line_center = (uint8_t)(64 + info->offset);
      const uint8_t marker_l4 = (line_center > 4U) ? (uint8_t)(line_center - 4U) : 0U;
      const uint8_t marker_l3 = (line_center > 3U) ? (uint8_t)(line_center - 3U) : 0U;
      const uint8_t marker_l2 = (line_center > 2U) ? (uint8_t)(line_center - 2U) : 0U;
      const uint8_t marker_l1 = (line_center > 1U) ? (uint8_t)(line_center - 1U) : 0U;
      const uint8_t marker_r4 = (line_center < 123U) ? (uint8_t)(line_center + 4U) : 127U;
      const uint8_t marker_r3 = (line_center < 124U) ? (uint8_t)(line_center + 3U) : 127U;
      const uint8_t marker_r2 = (line_center < 125U) ? (uint8_t)(line_center + 2U) : 127U;
      const uint8_t marker_r1 = (line_center < 126U) ? (uint8_t)(line_center + 1U) : 127U;
      fb_hline(marker_l4, marker_r4, 24);
      fb_hline(marker_l3, marker_r3, 25);
      fb_hline(marker_l2, marker_r2, 26);
      fb_hline(marker_l1, marker_r1, 27);
      fb_pixel(line_center, 28, 1);
      fb_vline(line_center, 30, 62);
      fb_hline(info->left_edge, info->right_edge, 56);
      fb_hline(info->left_edge, info->right_edge, 57);
    }
    else
    {
      fb_text(48, 28, "NO LINE");
    }

    (void)snprintf(line, sizeof(line), "MV%c O%03d %c%03u",
                   (info->fresh != 0U) ? 'K' : '-',
                   info->offset,
                   road_code(info->road_type),
                   info->confidence);
    fb_text(0, 16, line);
    (void)snprintf(line, sizeof(line), "%02u.%01uV", vin_x100 / 100U, (vin_x100 % 100U) / 10U);
    fb_text(104, 16, line);
  }

  fb_flush();
}

void Display_ShowRadar(const RadarSample *sample, RadarSide side, const char *state)
{
  char line[18];
  uint8_t bar;
  uint8_t score = 0;

  fb_clear();
  fb_text(0, 0, state);

  if ((sample != NULL) && (sample->valid != 0U))
  {
    score = (uint8_t)(sample->moving_energy + sample->static_energy);
    if (score > 100U)
    {
      score = 100U;
    }
    bar = (uint8_t)((score * 120U) / 100U);
    fb_hline(0, bar, 30);
    fb_hline(0, bar, 31);
    (void)snprintf(line, sizeof(line), "D%03u E%03u", sample->detect_distance_cm, score);
    fb_text(0, 10, line);
  }
  else
  {
    fb_text(0, 10, "NO RADAR");
  }

  if (side == RADAR_SIDE_LEFT)
  {
    fb_text(0, 44, "GO LEFT");
  }
  else if (side == RADAR_SIDE_RIGHT)
  {
    fb_text(0, 44, "GO RIGHT");
  }
  else
  {
    fb_text(0, 44, "UNKNOWN");
  }

  fb_flush();
}

void Display_ShowLora(uint8_t checkpoint, uint8_t sent, uint32_t elapsed_ms)
{
  char line[18];
  const uint32_t elapsed_s = elapsed_ms / 1000U;

  fb_clear();
  (void)snprintf(line, sizeof(line), "LORA 2-%u", checkpoint);
  fb_text(0, 0, line);
  fb_text(0, 14, sent ? "SENT" : "SEND?");
  (void)snprintf(line, sizeof(line), "T%02lu:%02lu", elapsed_s / 60U, elapsed_s % 60U);
  fb_text(0, 28, line);
  fb_flush();
}
