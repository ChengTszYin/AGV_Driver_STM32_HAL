#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define MATH_RAD_TO_DEG 57.2957795f
#define MATH_DEG_TO_MS 0.01745329251f
#define MATH_MS_TO_RPM 9.54929658551f
#define MATH_RPM_TO_MS 0.10471975511f

#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))
#define _round(x) ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x)-0.5f))
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

uint8_t fast_cal_crc8_maxim(uint8_t *p, uint8_t len);
