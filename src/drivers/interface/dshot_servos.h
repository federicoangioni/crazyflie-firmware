/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * dshot_servos.h - Servo PWM on M1/M3 while M2/M4 run DSHOT on TIM2
 */
#ifndef __DSHOT_SERVOS_H__
#define __DSHOT_SERVOS_H__

#include <stdint.h>
#include "motors.h"

/**
 * Take over the M1 (PA1) and M3 (PA15) pins from TIM2 and drive them with
 * 400 Hz, 1-2 ms servo PWM. M1 uses TIM5_CH2, M3 is bit-banged from TIM6.
 * Must be called after the DSHOT setup in motorsInit().
 */
void dshotServosInit(const MotorPerifDef** motorMap);

/**
 * Set the servo position for M1 or M3, same scaling as the standard PWM driver
 * (0 -> 1 ms, UINT16_MAX -> 2 ms). Calls for other motors are ignored.
 */
void dshotServosSetRatio(uint32_t id, uint16_t ratio);

#endif /* __DSHOT_SERVOS_H__ */
