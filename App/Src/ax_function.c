#include "ax_function.h"
#include "app_config.h"
#include "ax_ccd.h"
#include "ax_robot.h"

typedef struct
{
  AX_CCD_SegmentPreference preference;
  int16_t speed_limit;
} LineRouteStep;

static const LineRouteStep line_route_steps[] = {
  { AX_CCD_SEGMENT_LEFT,    APP_LINE_EXIT_SPEED_MM_S },  /* 1.1: enter the left U loop. */
  { AX_CCD_SEGMENT_RIGHT,   APP_LINE_EXIT_SPEED_MM_S },  /* 1.2: leave 1.1 and return to the main course. */
  { AX_CCD_SEGMENT_NEAREST, APP_LINE_LOOP_SPEED_MM_S },  /* 1.3: square/box area, follow the selected center line. */
  { AX_CCD_SEGMENT_NEAREST, APP_LINE_LOOP_SPEED_MM_S },  /* 1.4: circle/rectangle area, keep visual center tracking. */
  { AX_CCD_SEGMENT_NEAREST, APP_LINE_MIN_SPEED_MM_S  },  /* 1.5: approach finish. */
};

static uint8_t line_route_step;
static uint8_t line_route_event_armed = 1U;
static uint8_t line_route_event_ticks;
static uint32_t line_route_lock_until_ms;
static AX_CCD_SegmentPreference line_route_preference = AX_CCD_SEGMENT_NEAREST;
static int16_t line_route_speed_limit = APP_LINE_SPEED_MM_S;

static int16_t int16_min(int16_t a, int16_t b)
{
  return (a < b) ? a : b;
}

static uint8_t vision_is_route_event(const AX_CCD_LineInfo *info)
{
  if (info->line_valid == 0U)
  {
    return 0U;
  }

  if (info->confidence < APP_OPENMV_ROUTE_MIN_CONFIDENCE)
  {
    return 0U;
  }

  if (info->marker_detected == 0U)
  {
    return 0U;
  }

  if (info->segment_count >= 2U)
  {
    return 1U;
  }

  switch ((AX_VisionRoadType)info->road_type)
  {
    case AX_VISION_ROAD_FORK:
    case AX_VISION_ROAD_CROSS:
    case AX_VISION_ROAD_T_LEFT:
    case AX_VISION_ROAD_T_RIGHT:
      return 1U;
    default:
      return 0U;
  }
}

static void line_route_reset(void)
{
  line_route_step = 0U;
  line_route_event_armed = 1U;
  line_route_event_ticks = 0U;
  line_route_lock_until_ms = 0U;
  line_route_preference = AX_CCD_SEGMENT_NEAREST;
  line_route_speed_limit = APP_LINE_SPEED_MM_S;
}

uint8_t AX_FUN_IsFinishStage(void)
{
  return (line_route_step >= (uint8_t)(sizeof(line_route_steps) / sizeof(line_route_steps[0]))) ? 1U : 0U;
}

static void line_route_update(const AX_CCD_LineInfo *info)
{
  const uint32_t now = HAL_GetTick();
  uint8_t route_event = vision_is_route_event(info);

  line_route_preference = AX_CCD_SEGMENT_NEAREST;
  line_route_speed_limit = APP_LINE_SPEED_MM_S;

  if (route_event != 0U)
  {
    if (line_route_event_ticks < APP_OPENMV_ROUTE_CONFIRM_TICKS)
    {
      line_route_event_ticks++;
    }
  }
  else
  {
    line_route_event_ticks = 0U;
  }

  route_event = (line_route_event_ticks >= APP_OPENMV_ROUTE_CONFIRM_TICKS) ? 1U : 0U;

  if ((route_event == 0U) && (now >= line_route_lock_until_ms))
  {
    line_route_event_armed = 1U;
  }

  if (now < line_route_lock_until_ms)
  {
    if (line_route_step > 0U)
    {
      const uint8_t active_step = (uint8_t)(line_route_step - 1U);
      line_route_preference = line_route_steps[active_step].preference;
      line_route_speed_limit = line_route_steps[active_step].speed_limit;
    }
    return;
  }

  if ((route_event != 0U) &&
      (line_route_event_armed != 0U) &&
      (line_route_step < (uint8_t)(sizeof(line_route_steps) / sizeof(line_route_steps[0]))))
  {
    line_route_preference = line_route_steps[line_route_step].preference;
    line_route_speed_limit = line_route_steps[line_route_step].speed_limit;
    line_route_step++;
    line_route_event_armed = 0U;
    line_route_event_ticks = 0U;
    line_route_lock_until_ms = now + APP_OPENMV_ROUTE_LOCK_MS;
  }
}

static int16_t line_velocity_limit(int16_t base_velocity, const AX_CCD_LineInfo *info)
{
  int16_t velocity = int16_min(base_velocity, line_route_speed_limit);
  int16_t offset_abs;

  if (info->line_valid == 0U)
  {
    return APP_LINE_MIN_SPEED_MM_S;
  }

  if (((AX_VisionRoadType)info->road_type == AX_VISION_ROAD_LEFT_CURVE) ||
      ((AX_VisionRoadType)info->road_type == AX_VISION_ROAD_RIGHT_CURVE) ||
      ((AX_VisionRoadType)info->road_type == AX_VISION_ROAD_FORK) ||
      ((AX_VisionRoadType)info->road_type == AX_VISION_ROAD_CROSS))
  {
    velocity = int16_min(velocity, APP_LINE_LOOP_SPEED_MM_S);
  }

  offset_abs = info->offset;
  if (offset_abs < 0)
  {
    offset_abs = (int16_t)-offset_abs;
  }

  if (offset_abs > APP_LINE_SLOW_OFFSET)
  {
    velocity = (int16_t)(velocity -
                         ((offset_abs - APP_LINE_SLOW_OFFSET) * APP_LINE_SLOWDOWN_MM_S_PER_PIXEL));
  }

  if (velocity < APP_LINE_MIN_SPEED_MM_S)
  {
    velocity = APP_LINE_MIN_SPEED_MM_S;
  }

  return velocity;
}

static int16_t line_velocity_ramp(int16_t current_velocity, int16_t target_velocity)
{
  if (current_velocity < target_velocity)
  {
    current_velocity = (int16_t)(current_velocity + APP_LINE_SPEED_STEP_MM_S);
    if (current_velocity > target_velocity)
    {
      current_velocity = target_velocity;
    }
  }
  else if (current_velocity > target_velocity)
  {
    current_velocity = (int16_t)(current_velocity - APP_LINE_SPEED_STEP_MM_S);
    if (current_velocity < target_velocity)
    {
      current_velocity = target_velocity;
    }
  }

  return current_velocity;
}

void AX_FUN_Ls1(void)
{
  static float bias_last;
  static float last_valid_bias;
  static int16_t velocity_cmd;
  static uint8_t route_reset_done;
  static uint8_t lost_ticks;
  static uint8_t search_ticks;
  const int32_t distance_mm = AX_ROBOT_GetDistanceMm();
  float bias;
  float move_w;
  AX_CCD_LineInfo info;
  int16_t velocity_target;

  if ((distance_mm < 50) && (route_reset_done == 0U))
  {
    line_route_reset();
    velocity_cmd = 0;
    bias_last = 0.0f;
    last_valid_bias = 0.0f;
    lost_ticks = 0U;
    search_ticks = 0U;
    route_reset_done = 1U;
  }
  else if (distance_mm > 200)
  {
    route_reset_done = 0U;
  }

  (void)AX_CCD_GetOffset();
  info = AX_CCD_GetLineInfo();
  line_route_update(&info);
  AX_CCD_SetSegmentPreference(line_route_preference);

  if (info.line_valid == 0U)
  {
    lost_ticks++;
    if (lost_ticks <= APP_OPENMV_LOST_HOLD_TICKS)
    {
      bias = last_valid_bias;
      velocity_target = APP_LINE_MIN_SPEED_MM_S;
    }
    else if (search_ticks < APP_OPENMV_SEARCH_TICKS)
    {
      search_ticks++;
      velocity_cmd = line_velocity_ramp(velocity_cmd, APP_OPENMV_SEARCH_SPEED_MM_S);
      R_Vel.TG_IX = velocity_cmd;
      R_Vel.TG_IW = (last_valid_bias >= 0.0f) ? -APP_OPENMV_SEARCH_YAW : APP_OPENMV_SEARCH_YAW;
      ax_ccd_offset = (int16_t)last_valid_bias;
      return;
    }
    else
    {
      velocity_cmd = 0;
      R_Vel.TG_IX = 0;
      R_Vel.TG_IW = 0;
      ax_ccd_offset = (int16_t)last_valid_bias;
      return;
    }
  }
  else
  {
    lost_ticks = 0U;
    search_ticks = 0U;
    bias = (float)info.offset;
    last_valid_bias = bias;
    velocity_target = line_velocity_limit(ax_ccd_velocity, &info);
  }

  velocity_cmd = line_velocity_ramp(velocity_cmd, velocity_target);
  R_Vel.TG_IX = velocity_cmd;
  ax_ccd_offset = (int16_t)bias;

  move_w = -ax_ccd_kp * bias * 0.1f - ax_ccd_kd * (bias - bias_last) * 0.1f;
  R_Vel.TG_IW = (int16_t)move_w;

  bias_last = bias;
}
