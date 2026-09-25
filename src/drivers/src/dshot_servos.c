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
 * dshot_servos.c - Servo PWM on M1/M3 while M2/M4 run DSHOT on TIM2
 *
 * With DSHOT, TIM2 runs at the DSHOT bit rate (and in input capture for
 * bidirectional DSHOT), so it can not generate servo PWM on M1/M3 anymore.
 *
 * - M1 (PA1) is remapped to TIM5_CH2 (AF2) and gets hardware PWM.
 * - M3 (PA15) has no timer other than TIM2, so it is switched to a plain GPIO
 *   output and bit-banged from the TIM6 update interrupt. The interrupt runs
 *   above configMAX_SYSCALL_INTERRUPT_PRIORITY so FreeRTOS critical sections
 *   do not add jitter; it must therefore never call any FreeRTOS function.
 *
 * M1 and M3 stay in the DSHOT motor map: the bidirectional DSHOT sequencing
 * waits for their DMA transfers (M2 is started from the M1 DMA interrupt), so
 * TIM2_CH1/CH2 keep running internally without being connected to a pin.
 *
 * TIM5 is shared with the buzzer deck, the rpm deck and the servo deck on
 * TX2/RX2, which must therefore be disabled when this driver is used.
 *
 * Only the Bolt motor maps support DSHOT, and all of them have M1 on PA1 and
 * M3 on PA15 with an active high output, so this is hardcoded.
 */
#define DEBUG_MODULE "DSHOT-SRV"

#include <stdint.h>

/* ST includes */
#include "stm32fxxx.h"

#include "dshot_servos.h"
#include "nvicconf.h"
#include "cfassert.h"

#if defined(CONFIG_DECK_BUZZ) || defined(CONFIG_DECK_RPM) || \
    defined(CONFIG_DECK_SERVO_USE_TX2) || defined(CONFIG_DECK_SERVO_USE_RX2)
#error "TIM5 drives the M1 servo: disable the buzzer deck, rpm deck and servo deck on TX2/RX2"
#endif

// Same timing as the standard 400 Hz brushless PWM (PWM400), 1 us per tick
#define SERVO_TIM_PRESCALE     (uint16_t)(TIM_CLOCK_HZ / 1000000 - 1)
#define SERVO_PERIOD_US        2500
#define SERVO_PULSE_MIN_US     1000
#define SERVO_PULSE_RANGE_US   1000

// Pulse width in us requested for M3, 0 keeps the output low
static volatile uint16_t m3PulseUs = 0;
// Pulse width being output on M3, 0 when the output is low
static uint16_t m3ActivePulseUs = 0;

static void servoTimerInit(TIM_TypeDef* tim)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = SERVO_TIM_PRESCALE;
  TIM_TimeBaseStructure.TIM_Period = SERVO_PERIOD_US - 1;
  TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

void dshotServosInit(const MotorPerifDef** motorMap)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  ASSERT(motorMap[MOTOR_M1]->gpioPort == GPIOA && motorMap[MOTOR_M1]->gpioPin == GPIO_Pin_1);
  ASSERT(motorMap[MOTOR_M3]->gpioPort == GPIOA && motorMap[MOTOR_M3]->gpioPin == GPIO_Pin_15);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 | RCC_APB1Periph_TIM6, ENABLE);

  // M1: hardware PWM on TIM5_CH2, no pulse until the first set ratio
  servoTimerInit(TIM5);
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC2Init(TIM5, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_Cmd(TIM5, ENABLE);

  // Move PA1 from TIM2_CH2 to TIM5_CH2, and make PA15 a GPIO output (low)
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

  GPIOA->BSRRH = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // M3: TIM6 update interrupt toggles PA15. ARR is rewritten from the interrupt
  // for the next phase, so it must take effect immediately (no preload).
  servoTimerInit(TIM6);
  TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

  NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_SERVO_BITBANG_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM6, ENABLE);
}

void dshotServosSetRatio(uint32_t id, uint16_t ratio)
{
  const uint16_t pulse = SERVO_PULSE_MIN_US + ((uint32_t)ratio * SERVO_PULSE_RANGE_US) / UINT16_MAX;

  if (id == MOTOR_M1)
  {
    TIM_SetCompare2(TIM5, pulse);
  }
  else if (id == MOTOR_M3)
  {
    m3PulseUs = pulse;
  }
}

/**
 * Alternates between the high phase (pulse width) and the low phase (rest of
 * the period). The counter restarts from 0 in hardware at each update event,
 * so the interrupt latency only delays the pin edges, it does not accumulate.
 */
void __attribute__((used)) TIM6_DAC_IRQHandler(void)
{
  TIM6->SR = (uint16_t)~TIM_IT_Update;

  if (m3ActivePulseUs != 0)
  {
    GPIOA->BSRRH = GPIO_Pin_15;
    TIM6->ARR = SERVO_PERIOD_US - m3ActivePulseUs - 1;
    m3ActivePulseUs = 0;
  }
  else
  {
    m3ActivePulseUs = m3PulseUs;
    if (m3ActivePulseUs != 0)
    {
      GPIOA->BSRRL = GPIO_Pin_15;
      TIM6->ARR = m3ActivePulseUs - 1;
    }
    else
    {
      TIM6->ARR = SERVO_PERIOD_US - 1;
    }
  }
}
