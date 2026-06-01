#include "app_state.h"
#include "app_config.h"
#include "app_start.h"
#include "app_time.h"
#include "ax_kinematics.h"
#include "ax_robot.h"
#include "ax_vin.h"
#include "display_ssd1309.h"
#include "lora.h"
#include "maix_link.h"
#include "radar.h"

static AppStateId app_state = APP_STATE_IDLE;
static uint32_t last_display_ms;
static uint8_t pending_checkpoint;
static uint8_t checkpoint_sent_mask;
static uint8_t latest_battery_percent;

static uint32_t now_ms(void)
{
  return osKernelGetTickCount();
}

static const char *state_name(AppStateId state)
{
  switch (state)
  {
    case APP_STATE_IDLE: return "IDLE";
    case APP_STATE_RUN: return "RUN";
    case APP_STATE_LORA: return "LORA";
    case APP_STATE_FINISH: return "FINISH";
    case APP_STATE_FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

static void enter_state(AppStateId next)
{
  app_state = next;
}

static int16_t clamp_i16(int16_t value, int16_t min_value, int16_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static void set_velocity(int16_t vx, int16_t iw)
{
  R_Vel.TG_IX = clamp_i16(vx, (int16_t)(0 - APP_MAIX_MAX_SPEED_MM_S), APP_MAIX_MAX_SPEED_MM_S);
  R_Vel.TG_IY = 0;
  R_Vel.TG_IW = clamp_i16(iw, (int16_t)(0 - APP_MAIX_MAX_YAW), APP_MAIX_MAX_YAW);
}

static void stop_motion(void)
{
  ax_robot_move_enable = 0U;
  set_velocity(0, 0);
}

static void update_status_display(void)
{
  const uint32_t now = now_ms();
  char current_time[9];

  if ((now - last_display_ms) >= APP_DISPLAY_PERIOD_MS)
  {
    AppTime_GetBeijingTime(current_time, sizeof(current_time));
    Display_ShowStatus(state_name(app_state), AppStart_ElapsedMs(), latest_battery_percent,
                       current_time);
    last_display_ms = now;
  }
}

static void update_telemetry(uint8_t fault)
{
  const RadarSample sample = Radar_GetSample();
  uint8_t battery_percent;

  R_Bat_Vol = AX_VIN_GetVol_X100();
  battery_percent = AX_VIN_GetPercent(R_Bat_Vol);
  latest_battery_percent = battery_percent;
  MaixLink_TaskPoll(&sample,
                    AX_ROBOT_GetDistanceMm(),
                    R_Bat_Vol,
                    battery_percent,
                    AppStart_IsStarted(),
                    ax_robot_move_enable,
                    fault);
}

static uint8_t checkpoint_is_valid(uint8_t checkpoint)
{
  return ((checkpoint == 1U) || (checkpoint == 2U)) ? 1U : 0U;
}

static uint8_t checkpoint_already_sent(uint8_t checkpoint)
{
  const uint8_t bit = (uint8_t)(1U << checkpoint);
  return ((checkpoint_sent_mask & bit) != 0U) ? 1U : 0U;
}

static void mark_checkpoint_sent(uint8_t checkpoint)
{
  checkpoint_sent_mask |= (uint8_t)(1U << checkpoint);
}

static void update_lora_state(void)
{
  if (checkpoint_is_valid(pending_checkpoint) == 0U)
  {
    enter_state(APP_STATE_RUN);
    return;
  }

  MaixLink_SetLoraStatus(MAIX_LINK_LORA_SENDING);
  stop_motion();

  if (LoRa_SendCheckpoint(pending_checkpoint, AppStart_ElapsedMs()) == HAL_OK)
  {
    mark_checkpoint_sent(pending_checkpoint);
    MaixLink_SetLoraStatus(MAIX_LINK_LORA_SENT);
  }
  else
  {
    MaixLink_SetLoraStatus(MAIX_LINK_LORA_ERROR);
  }

  Display_ShowLora(pending_checkpoint,
                   checkpoint_already_sent(pending_checkpoint),
                   AppStart_ElapsedMs());
  pending_checkpoint = 0U;
  enter_state(APP_STATE_RUN);
}

static void apply_maix_command(const MaixLinkCommand *command)
{
  if (command == NULL)
  {
    stop_motion();
    return;
  }

  if (command->mode == MAIX_LINK_MODE_IDLE)
  {
    stop_motion();
    return;
  }

  ax_robot_move_enable = 1U;
  set_velocity(command->vx_mm_s, command->yaw);
}

static void update_run_state(void)
{
  MaixLinkCommand command;

  if (MaixLink_GetCommand(&command, APP_MAIX_LINK_TIMEOUT_MS) == 0U)
  {
    stop_motion();
    MaixLink_SetLoraStatus(MAIX_LINK_LORA_IDLE);
    enter_state(APP_STATE_FAULT);
    return;
  }

  if (command.mode == MAIX_LINK_MODE_FAULT)
  {
    stop_motion();
    enter_state(APP_STATE_FAULT);
    return;
  }

  if ((command.mode == MAIX_LINK_MODE_FINISH) ||
      ((command.flags & MAIX_LINK_CMD_FLAG_FINISH) != 0U))
  {
    stop_motion();
    enter_state(APP_STATE_FINISH);
    return;
  }

  if ((checkpoint_is_valid(command.checkpoint_request) != 0U) &&
      (checkpoint_already_sent(command.checkpoint_request) == 0U))
  {
    pending_checkpoint = command.checkpoint_request;
    MaixLink_SetLoraStatus(MAIX_LINK_LORA_IDLE);
    enter_state(APP_STATE_LORA);
    return;
  }

  apply_maix_command(&command);
  update_status_display();
}

void AppState_Init(void)
{
  Display_Init();
  R_Bat_Vol = AX_VIN_GetVol_X100();
  latest_battery_percent = AX_VIN_GetPercent(R_Bat_Vol);
  AppTime_Init();
  if (AppTime_IsValid() == 0U)
  {
    Display_ShowStatus("RTC ERR", 0, latest_battery_percent, "--:--:--");
    osDelay(500);
  }
  LoRa_Init();
  Radar_Init();
  MaixLink_Init();
  AppStart_Init();

  app_state = APP_STATE_IDLE;
  last_display_ms = 0U;
  pending_checkpoint = 0U;
  checkpoint_sent_mask = 0U;
  ax_robot_move_enable = 0U;
  MaixLink_SetLoraStatus(MAIX_LINK_LORA_IDLE);
  AX_ROBOT_ResetDistance();
}

uint8_t AppState_IsLineControlEnabled(void)
{
  return (app_state == APP_STATE_RUN) ? 1U : 0U;
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
    update_telemetry((app_state == APP_STATE_FAULT) ? 1U : 0U);

    switch (app_state)
    {
      case APP_STATE_IDLE:
        stop_motion();
        update_status_display();
        if (AppStart_IsStarted() != 0U)
        {
          AX_ROBOT_ResetDistance();
          checkpoint_sent_mask = 0U;
          MaixLink_SetLoraStatus(MAIX_LINK_LORA_IDLE);
          enter_state(APP_STATE_RUN);
        }
        break;

      case APP_STATE_RUN:
        update_run_state();
        break;

      case APP_STATE_LORA:
        update_lora_state();
        break;

      case APP_STATE_FINISH:
        stop_motion();
        update_status_display();
        break;

      case APP_STATE_FAULT:
      default:
        stop_motion();
        update_status_display();
        if ((AppStart_IsStarted() != 0U) && (MaixLink_IsFresh(APP_MAIX_LINK_TIMEOUT_MS) != 0U))
        {
          enter_state(APP_STATE_RUN);
        }
        break;
    }

    osDelay(APP_STATE_PERIOD_MS);
  }
}
