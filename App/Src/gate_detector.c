#include "gate_detector.h"
#include "app_config.h"

typedef enum
{
  GATE_STATE_CLEAR = 0,
  GATE_STATE_CANDIDATE,
  GATE_STATE_UNDER_GATE,
  GATE_STATE_EXITING,
  GATE_STATE_LOCKED
} GateState;

static GateState gate_state;
static GateDetectorEvent pending_event;
static uint8_t gate_count;
static uint32_t state_since_ms;
static uint32_t lock_since_ms;
static int32_t lock_distance_mm;
static uint16_t last_distance_cm;
static uint16_t last_score;

static int16_t abs_i16(int16_t value)
{
  return (value < 0) ? (int16_t)(0 - value) : value;
}

static uint8_t gate_find_pair(const RadarSample *sample, uint16_t *distance_cm, uint16_t *score)
{
  uint8_t i;
  uint8_t j;
  uint8_t found = 0U;
  uint16_t best_distance_cm = 0U;

  if ((sample == NULL) || (sample->valid == 0U))
  {
    return 0U;
  }

  for (i = 0; i < RADAR_MAX_TARGETS; i++)
  {
    const RadarTarget *left = &sample->targets[i];
    if ((left->valid == 0U) || (left->x_mm >= 0) ||
        (abs_i16(left->x_mm) < APP_GATE_POST_MIN_ABS_X_MM) ||
        (left->y_mm < (int16_t)(APP_GATE_MIN_CM * 10)) ||
        (left->y_mm > (int16_t)(APP_GATE_MAX_CM * 10)))
    {
      continue;
    }

    for (j = 0; j < RADAR_MAX_TARGETS; j++)
    {
      const RadarTarget *right = &sample->targets[j];
      const int16_t width_mm = (int16_t)(right->x_mm - left->x_mm);

      if ((right->valid == 0U) || (right->x_mm <= 0) ||
          (abs_i16(right->x_mm) < APP_GATE_POST_MIN_ABS_X_MM) ||
          (right->y_mm < (int16_t)(APP_GATE_MIN_CM * 10)) ||
          (right->y_mm > (int16_t)(APP_GATE_MAX_CM * 10)) ||
          (abs_i16((int16_t)(left->y_mm - right->y_mm)) > APP_GATE_PAIR_Y_TOL_MM) ||
          (width_mm < APP_GATE_WIDTH_MIN_MM) || (width_mm > APP_GATE_WIDTH_MAX_MM))
      {
        continue;
      }

      best_distance_cm = (uint16_t)(((uint16_t)left->y_mm + (uint16_t)right->y_mm) / 20U);
      found = 1U;
      break;
    }

    if (found != 0U)
    {
      break;
    }
  }

  if (found != 0U)
  {
    if (distance_cm != NULL)
    {
      *distance_cm = best_distance_cm;
    }
    if (score != NULL)
    {
      *score = 80U;
    }
  }

  return found;
}

static uint16_t gate_score(const RadarSample *sample)
{
  uint16_t score;

  if ((sample == NULL) || (sample->valid == 0U))
  {
    return 0;
  }

  if (gate_find_pair(sample, NULL, &score) != 0U)
  {
    return score;
  }

  score = (uint16_t)sample->moving_energy + (uint16_t)sample->static_energy;
  if (sample->target_state != 0U)
  {
    score = (uint16_t)(score + 10U);
  }

  return score;
}

static uint16_t gate_distance_cm(const RadarSample *sample)
{
  uint16_t distance_cm = 0U;

  if ((sample == NULL) || (sample->valid == 0U))
  {
    return 0U;
  }

  if (gate_find_pair(sample, &distance_cm, NULL) != 0U)
  {
    return distance_cm;
  }

  return sample->detect_distance_cm;
}

static uint8_t gate_target_active(const RadarSample *sample, uint16_t score)
{
  const uint16_t distance_cm = gate_distance_cm(sample);

  if ((sample == NULL) || (sample->valid == 0U))
  {
    return 0;
  }

  if ((distance_cm < APP_GATE_MIN_CM) ||
      (distance_cm > APP_GATE_MAX_CM))
  {
    return 0;
  }

  return (score >= APP_GATE_MIN_SCORE) ? 1U : 0U;
}

static uint8_t gate_target_exiting(const RadarSample *sample, uint16_t score)
{
  const uint16_t distance_cm = gate_distance_cm(sample);

  if ((sample == NULL) || (sample->valid == 0U))
  {
    return 1;
  }

  if (score < APP_GATE_MIN_SCORE)
  {
    return 1;
  }

  if ((distance_cm < APP_GATE_MIN_CM) ||
      (distance_cm > APP_GATE_MAX_CM))
  {
    return 1;
  }

  if ((distance_cm > last_distance_cm) &&
      ((distance_cm - last_distance_cm) >= APP_GATE_EXIT_JUMP_CM))
  {
    return 1;
  }

  return 0;
}

static void gate_enter_state(GateState next, uint32_t now_ms)
{
  gate_state = next;
  state_since_ms = now_ms;
}

void GateDetector_Init(void)
{
  gate_state = GATE_STATE_CLEAR;
  pending_event = GATE_EVENT_NONE;
  gate_count = 0;
  state_since_ms = 0;
  lock_since_ms = 0;
  lock_distance_mm = 0;
  last_distance_cm = 0;
  last_score = 0;
}

void GateDetector_Update(const RadarSample *sample, uint32_t now_ms, int32_t distance_mm)
{
  const uint16_t score = gate_score(sample);
  const uint8_t active = gate_target_active(sample, score);

  last_score = score;

  switch (gate_state)
  {
    case GATE_STATE_CLEAR:
      if (active != 0U)
      {
        gate_enter_state(GATE_STATE_CANDIDATE, now_ms);
      }
      break;

    case GATE_STATE_CANDIDATE:
      if (active == 0U)
      {
        gate_enter_state(GATE_STATE_CLEAR, now_ms);
      }
      else if ((now_ms - state_since_ms) >= APP_GATE_ENTER_STABLE_MS)
      {
        gate_enter_state(GATE_STATE_UNDER_GATE, now_ms);
      }
      break;

    case GATE_STATE_UNDER_GATE:
      if (gate_target_exiting(sample, score) != 0U)
      {
        gate_enter_state(GATE_STATE_EXITING, now_ms);
      }
      break;

    case GATE_STATE_EXITING:
      if (active != 0U)
      {
        gate_enter_state(GATE_STATE_UNDER_GATE, now_ms);
      }
      else if ((now_ms - state_since_ms) >= APP_GATE_EXIT_STABLE_MS)
      {
        if (pending_event == GATE_EVENT_NONE)
        {
          pending_event = GATE_EVENT_PASSED;
          gate_count++;
        }
        lock_since_ms = now_ms;
        lock_distance_mm = distance_mm;
        gate_enter_state(GATE_STATE_LOCKED, now_ms);
      }
      break;

    case GATE_STATE_LOCKED:
      if (((now_ms - lock_since_ms) >= APP_GATE_LOCK_MS) &&
          ((distance_mm - lock_distance_mm) >= APP_GATE_LOCK_DISTANCE_MM))
      {
        gate_enter_state(GATE_STATE_CLEAR, now_ms);
      }
      break;

    default:
      gate_enter_state(GATE_STATE_CLEAR, now_ms);
      break;
  }

  if ((sample != NULL) && (sample->valid != 0U))
  {
    last_distance_cm = gate_distance_cm(sample);
  }
}

GateDetectorEvent GateDetector_GetEvent(void)
{
  const GateDetectorEvent event = pending_event;
  pending_event = GATE_EVENT_NONE;
  return event;
}

uint8_t GateDetector_GetCount(void)
{
  return gate_count;
}

const char *GateDetector_GetStateName(void)
{
  switch (gate_state)
  {
    case GATE_STATE_CLEAR: return "NO";
    case GATE_STATE_CANDIDATE: return "CAND";
    case GATE_STATE_UNDER_GATE: return "GATE";
    case GATE_STATE_EXITING: return "EXIT";
    case GATE_STATE_LOCKED: return "LOCK";
    default: return "UNK";
  }
}

uint16_t GateDetector_GetLastScore(void)
{
  return last_score;
}
