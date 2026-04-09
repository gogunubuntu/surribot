#include "config.h"
#ifdef LIDAR_USE_SIM

#include "lidar.h"
#include <math.h>

#define ROOM_W  4.0f
#define ROOM_H  3.0f
#define OBS_X   1.5f
#define OBS_Y   1.0f
#define OBS_R   0.3f

static float cast_ray(float angle) {
  float dx = cosf(angle), dy = sinf(angle);
  float dist = SCAN_RANGE_MAX;

  if (dx > 1e-6f)  { float t = (ROOM_W/2.0f)/dx;  if (fabsf(t*dy) <= ROOM_H/2.0f && t < dist) dist = t; }
  if (dx < -1e-6f) { float t = (-ROOM_W/2.0f)/dx; if (fabsf(t*dy) <= ROOM_H/2.0f && t < dist) dist = t; }
  if (dy > 1e-6f)  { float t = (ROOM_H/2.0f)/dy;  if (fabsf(t*dx) <= ROOM_W/2.0f && t < dist) dist = t; }
  if (dy < -1e-6f) { float t = (-ROOM_H/2.0f)/dy; if (fabsf(t*dx) <= ROOM_W/2.0f && t < dist) dist = t; }

  float b = 2.0f * (-OBS_X*dx - OBS_Y*dy);
  float c = OBS_X*OBS_X + OBS_Y*OBS_Y - OBS_R*OBS_R;
  float det = b*b - 4.0f*c;
  if (det >= 0.0f) { float t1 = (-b - sqrtf(det)) / 2.0f; if (t1 > SCAN_RANGE_MIN && t1 < dist) dist = t1; }
  return dist;
}

void lidar_init() {}

void lidar_update() {}

bool lidar_get_scan(float* ranges, int num_slots) {
  float angle_inc = (2.0f * M_PI) / num_slots;
  for (int i = 0; i < num_slots; i++) {
    float angle = i * angle_inc;
    float d = cast_ray(angle) + ((float)(random(-20, 21)) / 1000.0f);
    if (d < SCAN_RANGE_MIN) d = SCAN_RANGE_MIN;
    if (d > SCAN_RANGE_MAX) d = SCAN_RANGE_MAX;
    ranges[i] = d;
  }
  return true;
}

#endif
