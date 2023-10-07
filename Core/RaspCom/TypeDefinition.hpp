#pragma once

#include "main.h"

#define BALLINFO_LENGTH (sizeof(BallInformation))

typedef struct{
  float x;
  float y;
  float vx;
  float vy;
  uint32_t status;
} __attribute__((__packed__)) BallInformation;
