#include "app_state.h"
#include "app_config.h"
#include "app_start.h"
#include "app_time.h"
#include "ax_ccd.h"
#include "ax_function.h"
#include "ax_kinematics.h"
#include "ax_robot.h"
#include "ax_vin.h"
#include "display_ssd1309.h"
#include "gate_detector.h"
#include "lora.h"
#include "radar.h"

typedef enum
{
  RADAR_SCAN_COLLECT = 0,
  RADAR_SCAN_RESCAN,
  RADAR_SCAN_DONE
} RadarScanPhase;

typedef enum
{
  AVOID_PHASE_ENTRY = 0,
  AVOID_PHASE_PASS,
  AVOID_PHASE_RECOVER
} AvoidPhase;

static AppStateId app_state = APP_STATE_IDLE;
static uint32_t state_enter_ms;
static uint32_t last_display_ms;
static uint8_t lora_sent;
static RadarScanPhase radar_phase;
static uint32_t radar_phase_start_ms;
static int32_t radar_scan_start_distance_mm;
static uint8_t radar_rescan_used;
static uint8_t box_left_stable_frames;
static uint8_t box_right_stable_frames;
static uint16_t radar_left_score;
static uint16_t radar_right_score;
static RadarSide radar_decision = RADAR_SIDE_UNKNOWN;
static AvoidPhase avoid_phase;
static uint32_t avoid_phase_start_ms;

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
    radar_phase = RADAR_SCAN_COLLECT;
    radar_phase_start_ms = state_enter_ms;
    radar_scan_start_distance_mm = AX_ROBOT_GetDistanceMm();
    radar_rescan_used = 0U;
    box_left_stable_frames = 0U;
    box_right_stable_frames = 0U;
    radar_left_score = 0;
    radar_right_score = 0;
    radar_decision = RADAR_SIDE_UNKNOWN;
  }
  else if ((next == APP_STATE_AVOID_LEFT) || (next == APP_STATE_AVOID_RIGHT))
  {
    avoid_phase = AVOID_PHASE_ENTRY;
    avoid_phase_start_ms = state_enter_ms;
  }
}

static int16_t abs_i16(int16_t value)
{
  return (value < 0) ? (int16_t)(0 - value) : value;
}

static uint16_t box_target_weight(const RadarTarget *target)
{
  uint16_t score = 1U;

  if ((target == NULL) || (target->valid == 0U))
  {
    return 0U;
  }

  if ((target->y_mm < APP_BOX_Y_MIN_MM) || (target->y_mm > APP_BOX_Y_MAX_MM))
  {
    return 0U;
  }

  if (target->y_mm <= 900)
  {
    score = (uint16_t)(score + 2U);
  }
  else if (target->y_mm <= 1400)
  {
    score = (uint16_t)(score + 1U);
  }

  if (abs_i16(target->speed_cm_s) > 0)
  {
    score = (uint16_t)(score + 1U);
  }

  return score;
}

static void box_accumulate_scores(const RadarSample *sample)
{
  uint8_t i;
  uint16_t left_frame_score = 0U;
  uint16_t right_frame_score = 0U;

  if ((sample == NULL) || (sample->valid == 0U))
  {
    box_left_stable_frames = 0U;
    box_right_stable_frames = 0U;
    return;
  }

  for (i = 0; i < RADAR_MAX_TARGETS; i++)
  {
    const RadarTarget *target = &sample->targets[i];
    const uint16_t weight = box_target_weight(target);

    if (weight == 0U)
    {
      continue;
    }

    if ((target->x_mm >= APP_BOX_LEFT_X_MIN_MM) && (target->x_mm <= APP_BOX_LEFT_X_MAX_MM))
    {
      left_frame_score = (uint16_t)(left_frame_score + weight);
    }
    else if ((target->x_mm >= APP_BOX_RIGHT_X_MIN_MM) && (target->x_mm <= APP_BOX_RIGHT_X_MAX_MM))
    {
      right_frame_score = (uint16_t)(right_frame_score + weight);
    }
  }

  if (left_frame_score > 0U)
  {
    if (box_left_stable_frames < 0xFFU)
    {
      box_left_stable_frames++;
    }
  }
  else
  {
    box_left_stable_frames = 0U;
  }

  if (right_frame_score > 0U)
  {
    if (box_right_stable_frames < 0xFFU)
    {
      box_right_stable_frames++;
    }
  }
  else
  {
    box_right_stable_frames = 0U;
  }

  if (box_left_stable_frames >= APP_BOX_STABLE_FRAMES)
  {
    radar_left_score = (uint16_t)(radar_left_score + left_frame_score);
  }

  if (box_right_stable_frames >= APP_BOX_STABLE_FRAMES)
  {
    radar_right_score = (uint16_t)(radar_right_score + right_frame_score);
  }
}

static RadarSide box_decide_free_side(void)
{
  if (radar_left_score > (uint16_t)(radar_right_score + APP_BOX_SCORE_MARGIN))
  {
    return RADAR_SIDE_RIGHT;
  }

  if (radar_right_score > (uint16_t)(radar_left_score + APP_BOX_SCORE_MARGIN))
  {
    return RADAR_SIDE_LEFT;
  }

  return RADAR_SIDE_UNKNOWN;
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

  if (AX_FUN_IsFinishStage() == 0U)
  {
    return 0U;
  }

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
  const int32_t scan_distance = AX_ROBOT_GetDistanceMm() - radar_scan_start_distance_mm;
  const uint32_t phase_elapsed = now - radar_phase_start_ms;
  int16_t scan_yaw = (int16_t)(APP_BOX_RESCAN_YAW / 3);

  box_accumulate_scores(&sample);

  if (((now / 200U) & 1U) != 0U)
  {
    scan_yaw = (int16_t)(0 - scan_yaw);
  }

  if (radar_phase == RADAR_SCAN_COLLECT)
  {
    set_velocity(APP_RADAR_SCAN_SPEED_MM_S, scan_yaw);

    if ((scan_distance >= APP_BOX_SCAN_MIN_DISTANCE_MM) ||
        (phase_elapsed >= APP_BOX_SCAN_MAX_MS))
    {
      radar_decision = box_decide_free_side();
      if ((radar_decision == RADAR_SIDE_UNKNOWN) && (radar_rescan_used == 0U))
      {
        radar_phase = RADAR_SCAN_RESCAN;
        radar_phase_start_ms = now;
        radar_rescan_used = 1U;
      }
      else
      {
        if (radar_decision == RADAR_SIDE_UNKNOWN)
        {
          radar_decision = APP_BOX_DEFAULT_SIDE;
        }
        radar_phase = RADAR_SCAN_DONE;
      }
    }
  }
  else if (radar_phase == RADAR_SCAN_RESCAN)
  {
    const int16_t rescan_yaw = (phase_elapsed < (APP_BOX_RESCAN_MS / 2U)) ?
                               APP_BOX_RESCAN_YAW : (int16_t)(0 - APP_BOX_RESCAN_YAW);
    set_velocity(APP_RADAR_SCAN_SPEED_MM_S, rescan_yaw);

    if (phase_elapsed >= APP_BOX_RESCAN_MS)
    {
      radar_decision = box_decide_free_side();
      if (radar_decision == RADAR_SIDE_UNKNOWN)
      {
        radar_decision = APP_BOX_DEFAULT_SIDE;
      }
      radar_phase = RADAR_SCAN_DONE;
    }
  }

  if (radar_phase == RADAR_SCAN_DONE)
  {
    enter_state((radar_decision == RADAR_SIDE_LEFT) ? APP_STATE_AVOID_LEFT : APP_STATE_AVOID_RIGHT);
  }

  if ((now - last_display_ms) >= APP_DISPLAY_PERIOD_MS)
  {
    Display_ShowRadar(&sample, radar_decision, state_name(app_state));
    last_display_ms = now;
  }
}

static void update_avoid_state(void)
{
  const uint32_t now = now_ms();
  const int16_t turn_yaw = (app_state == APP_STATE_AVOID_LEFT) ? APP_AVOID_YAW : (int16_t)(0 - APP_AVOID_YAW);

  if (avoid_phase == AVOID_PHASE_ENTRY)
  {
    set_velocity(APP_AVOID_SPEED_MM_S, turn_yaw);
    if ((now - avoid_phase_start_ms) >= APP_AVOID_ENTRY_MS)
    {
      avoid_phase = AVOID_PHASE_PASS;
      avoid_phase_start_ms = now;
    }
  }
  else if (avoid_phase == AVOID_PHASE_PASS)
  {
    set_velocity(APP_AVOID_SPEED_MM_S, 0);
    if ((now - avoid_phase_start_ms) >= APP_AVOID_PASS_MS)
    {
      avoid_phase = AVOID_PHASE_RECOVER;
      avoid_phase_start_ms = now;
    }
  }
  else
  {
    set_velocity(APP_AVOID_SPEED_MM_S, (int16_t)(0 - (turn_yaw / 2)));
    if ((now - avoid_phase_start_ms) >= APP_AVOID_RECOVER_MS)
    {
      enter_state(APP_STATE_LINE_TASK);
      return;
    }
  }

  if ((now - last_display_ms) >= APP_DISPLAY_PERIOD_MS)
  {
    const RadarSample sample = Radar_GetSample();
    Display_ShowRadar(&sample, radar_decision, state_name(app_state));
    last_display_ms = now;
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
