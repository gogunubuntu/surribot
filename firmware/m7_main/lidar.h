#pragma once
#include <Arduino.h>
#include "config.h"

void lidar_init();
void lidar_update();
bool lidar_get_scan(float* ranges, int num_slots);
