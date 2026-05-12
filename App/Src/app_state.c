#include "app_state.h"
#include "app_config.h"
#include "app_start.h"
#include "app_time.h"
#include "ax_ccd.h"
#include "ax_kinematics.h"
#include "ax_robot.h"
#include "ax_vin.h"
#include "display_ssd1309.h"
#include "gate_detector.h"
#include "lora.h"
#include "radar.h"

typedef enum
{
  RADAR_SCAN_LEFT = 0,
  RADAR_SCAN_RIGHT,
  RADAR_SCAN_DONE
} RadarScanPhase;

static AppStateId app_state = APP_STATE_IDLE;
static uint32_t state_enter_ms;
static uint32_t last_display_ms;
static uint8_t lora_sent;
static RadarScanPhase radar_phase;
static uint32_t radar_phase_start_ms;
static uint16_t radar_left_score;
static uint16_t radar_right_score;
static RadarSide radar_decision = RADAR_SIDE_UNKNOWN;

static uint32_t now_ms(void)
{
  return osKernelGetTickCount();
}

static const char *state_name(AppStateId state)
{
  switch (state)
  {
    case APP_STATE_IDLE: return "IDLE";
    case APP_STATE_LINE_TASK: return "LINE";
    case APP_STATE_LORA_2_1: return "LORA21";
    case APP_STATE_LORA_2_2: return "LORA22";
    case APP_STATE_RADAR_PRE_SCAN: return "RADAR";
    case APP_STATE_AVOID_LEFT: return "LEFT";
    case APP_STATE_AVOID_RIGHT: return "RIGHT";
    case APP_STATE_FINISH: return "FINISH";
    default: return "UNKNOWN";
  }
}

static void enter_state(AppStateId next)
{
  app_state = next;
  state_enter_ms = now_ms();
  lora_sent = 0;

  if (next == APP_STATE_RADAR_PRE_SCAN)
  {
    radar_phase = RADAR_SCAN_LEFT;
    radar_phase_start_ms = state_enter_ms;
    radar_left_score = 0;
    radar_right_score = 0;
    radar_decision = RADAR_SIDE_UNKNOWN;
  }
}

static uint16_t radar_score(RadarSample sample)
{
  if (sample.valid == 0U)
  {
    return 0;
  }

  return (uint16_t)sample.moving_energy + (uint16_t)sample.static_energy +
         (sample.target_state != 0U ? 10U : 0U);
}

static void set_velocity(int16_t vx, int16_t iw)
{
  R_Vel.TG_IX = vx;
  R_Vel.TG_IY = 0;
  R_Vel.TG_IW = iw;
}

static void handle_gate_detection(void)
{
  const RadarSample sample = Radar_GetSample();
  const int32_t distance = AX_ROBOT_GetDistanceMm();

  GateDetector_Update(&sample, now_ms(), distance);

  if (GateDetector_GetEvent() == GATE_EVENT_PASSED)
  {
    const uint8_t gate_count = GateDetector_GetCount();

    if (gate_count == 1U)
    {
      enter_state(APP_STATE_LORA_2_1);
    }
    else if (gate_count == 2U)
    {
      enter_state(APP_STATE_LORA_2_2);
    }
    else if (gate_count == 3U)
    {
      enter_state(APP_STATE_RADAR_PRE_SCAN);
    }
  }
}

static void update_line_display(void)
{
  const uint32_t now = now_ms();
  if ((now - last_display_ms) >= APP_DISPLAY_PERIOD_MS)
  {
    const AX_CCD_LineInfo info = AX_CCD_GetLineInfo();
    const RadarSample sample = Radar_GetSample();
    uint8_t battery_percent;
    R_Bat_Vol = AX_VIN_GetVol_X100();
    battery_percent = AX_VIN_GetPercent(R_Bat_Vol);
    Display_ShowLineGate(AX_CCD_GetLastFrame(), &info, &sample, state_name(app_state),
                         GateDetector_GetCount(), GateDetector_GetStateName(),
                         GateDetector_GetLastScore(), R_Bat_Vol, battery_percent);
    last_display_ms = now;
  }
}

static uint8_t vision_finish_detected(void)
{
  const AX_CCD_LineInfo info = AX_CCD_GetLineInfo();

  return ((info.finish_detected != 0U) ||
          ((AX_VisionRoadType)info.road_type == AX_VISION_ROAD_FINISH)) ? 1U : 0U;
}

static void update_status_display(void)
{
  const uint32_t now = now_ms();
  if ((now - last_display_ms) >= APP_DISPLAY_PERIOD_MS)
  {
    Display_ShowStatus(state_name(app_state), AppStart_ElapsedMs());
    last_display_ms = now;
  }
}

static void update_lora_state(uint8_t checkpoint)
{
  if (lora_sent == 0U)
  {
    if (LoRa_SendCheckpoint(checkpoint, AppStart_ElapsedMs()) == HAL_OK)
    {
      lora_sent = 1;
    }
  }

  Display_ShowLora(checkpoint, lora_sent, AppStart_ElapsedMs());

  if ((now_ms() - state_enter_ms) > 800U)
  {
    enter_state(APP_STATE_LINE_TASK);
  }
}

static void update_radar_scan(void)
{
  const uint32_t now = now_ms();
  const RadarSample sample = Radar_GetSample();
  const uint16_t score = radar_score(sample);

  if (radar_phase == RADAR_SCAN_LEFT)
  {
    set_velocity(0, APP_RADAR_SCAN_YAW);
    if (score > radar_left_score)
    {
      radar_left_score = score;
    }

    if ((now - radar_phase_start_ms) > APP_RADAR_SCAN_MS)
    {
      radar_phase = RADAR_SCAN_RIGHT;
      radar_phase_start_ms = now;
    }
  }
  else if (radar_phase == RADAR_SCAN_RIGHT)
  {
    set_velocity(0, -APP_RADAR_SCAN_YAW);
    if (score > radar_right_score)
    {
      radar_right_score = score;
    }

    if ((now - radar_phase_start_ms) > (APP_RADAR_SCAN_MS * 2U))
    {
      radar_phase = RADAR_SCAN_DONE;
      radar_decision = (radar_left_score <= radar_right_score) ? RADAR_SIDE_LEFT : RADAR_SIDE_RIGHT;
      enter_state((radar_decision == RADAR_SIDE_LEFT) ? APP_STATE_AVOID_LEFT : APP_STATE_AVOID_RIGHT);
    }
  }

  if ((now - last_display_ms) >= APP_DISPLAY_PERIOD_MS)
  {
    Display_ShowRadar(&sample, radar_decision, state_name(app_state));
    last_display_ms = now;
  }
}

static void update_avoid_state(void)
{
  const int16_t yaw = (app_state == APP_STATE_AVOID_LEFT) ? APP_AVOID_YAW : -APP_AVOID_YAW;
  set_velocity(APP_AVOID_SPEED_MM_S, yaw);

  if ((now_ms() - last_display_ms) >= APP_DISPLAY_PERIOD_MS)
  {
    const RadarSample sample = Radar_GetSample();
    Display_ShowRadar(&sample, radar_decision, state_name(app_state));
    last_display_ms = now_ms();
  }

  if ((now_ms() - state_enter_ms) > APP_AVOID_MS)
  {
    enter_state(APP_STATE_LINE_TASK);
  }
}

void AppState_Init(void)
{
  Display_Init();
  AppTime_Init();
  if (AppTime_IsValid() == 0U)
  {
    Display_ShowStatus("RTC ERR", 0);
    osDelay(500);
  }
  LoRa_Init();
  Radar_Init();
  GateDetector_Init();
  AppStart_Init();

  app_state = APP_STATE_IDLE;
  state_enter_ms = now_ms();
  last_display_ms = 0;
  lora_sent = 0;
  radar_decision = RADAR_SIDE_UNKNOWN;
  ax_robot_move_enable = 0;
  AX_ROBOT_ResetDistance();
}

uint8_t AppState_IsLineControlEnabled(void)
{
  return (app_state == APP_STATE_LINE_TASK) ? 1U : 0U;
}

AppStateId AppState_Get(void)
{
  return app_state;
}

void AppState_Task(void *argument)
{
  (void)argument;
  AppState_Init();

  for (;;)
  {
    AppStart_Poll();
    AppTime_TaskPoll();
    Radar_TaskPoll();

    switch (app_state)
    {
      case APP_STATE_IDLE:
        ax_robot_move_enable = 0;
        set_velocity(0, 0);
        (void)AX_CCD_GetOffset();
        update_line_display();
        if (AppStart_IsStarted() != 0U)
        {
          AX_ROBOT_ResetDistance();
          GateDetector_Init();
          ax_robot_move_enable = 1;
          ax_ccd_velocity = APP_LINE_SPEED_MM_S;
          enter_state(APP_STATE_LINE_TASK);
        }
        break;

      case APP_STATE_LINE_TASK:
        ax_robot_move_enable = 1;
        handle_gate_detection();
        if (vision_finish_detected() != 0U)
        {
          enter_state(APP_STATE_FINISH);
        }
        update_line_display();
        break;

      case APP_STATE_LORA_2_1:
        ax_robot_move_enable = 0;
        set_velocity(0, 0);
        update_lora_state(1);
        break;

      case APP_STATE_LORA_2_2:
        ax_robot_move_enable = 0;
        set_velocity(0, 0);
        update_lora_state(2);
        break;

      case APP_STATE_RADAR_PRE_SCAN:
        ax_robot_move_enable = 1;
        update_radar_scan();
        break;

      case APP_STATE_AVOID_LEFT:
      case APP_STATE_AVOID_RIGHT:
        ax_robot_move_enable = 1;
        update_avoid_state();
        break;

      case APP_STATE_FINISH:
      default:
        ax_robot_move_enable = 0;
        set_velocity(0, 0);
        update_status_display();
        break;
    }

    osDelay(APP_STATE_PERIOD_MS);
  }
}
