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

static uint16_t gate_score(const RadarSample *sample)
{
  uint16_t score;

  if ((sample == NULL) || (sample->valid == 0U))
  {
    return 0;
  }

  score = (uint16_t)sample->moving_energy + (uint16_t)sample->static_energy;
  if (sample->target_state != 0U)
  {
    score = (uint16_t)(score + 10U);
  }

  return score;
}

static uint8_t gate_target_active(const RadarSample *sample, uint16_t score)
{
  if ((sample == NULL) || (sample->valid == 0U))
  {
    return 0;
  }

  if ((sample->detect_distance_cm < APP_GATE_MIN_CM) ||
      (sample->detect_distance_cm > APP_GATE_MAX_CM))
  {
    return 0;
  }

  return (score >= APP_GATE_MIN_SCORE) ? 1U : 0U;
}

static uint8_t gate_target_exiting(const RadarSample *sample, uint16_t score)
{
  if ((sample == NULL) || (sample->valid == 0U))
  {
    return 1;
  }

  if (score < APP_GATE_MIN_SCORE)
  {
    return 1;
  }

  if ((sample->detect_distance_cm < APP_GATE_MIN_CM) ||
      (sample->detect_distance_cm > APP_GATE_MAX_CM))
  {
    return 1;
  }

  if ((sample->detect_distance_cm > last_distance_cm) &&
      ((sample->detect_distance_cm - last_distance_cm) >= APP_GATE_EXIT_JUMP_CM))
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
    last_distance_cm = sample->detect_distance_cm;
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
