/* * OSS-7 - An opensource implementation of the DASH7 Alliance Protocol for ultra
 * lowpower wireless sensor communication
 *
 * Copyright 2015 University of Antwerp
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _GPIO_H
#define _GPIO_H
#include <machine/sfradr.h>

typedef struct GPio
{
    volatile unsigned out;
    volatile unsigned in;
    volatile unsigned dir;
    volatile unsigned old_in;
    volatile unsigned mask;
} GPio;


/** GPIO ports identificator. */
typedef enum
{
  gpioPortA = SFRADR_GPIO1, /**< Port A */
  gpioPortB = SFRADR_GPIO1, /**< Port B */
  //gpioPortC = 2, /**< Port C */
  //gpioPortD = 3, /**< Port D */
  //gpioPortE = 4, /**< Port E */
  //gpioPortF = 5  /**< Port F */
} GPIO_Port_TypeDef;

#if 0
/** GPIO drive mode. */
typedef enum
{
  /** Default 6mA */
  gpioDriveModeStandard = GPIO_P_CTRL_DRIVEMODE_STANDARD,
  /** 0.5 mA */
  gpioDriveModeLowest   = GPIO_P_CTRL_DRIVEMODE_LOWEST,
  /** 20 mA */
  gpioDriveModeHigh     = GPIO_P_CTRL_DRIVEMODE_HIGH,
  /** 2 mA */
  gpioDriveModeLow      = GPIO_P_CTRL_DRIVEMODE_LOW
} GPIO_DriveMode_TypeDef;
#endif

/** Pin mode. For more details on each mode, please refer to the EFM32
 * reference manual. */
typedef enum
{
  /** Input enabled. Filter if DOUT is set */
  gpioModeInput                     = 0,
  /** Push-pull output */
  gpioModePushPull                  = 1

} GPIO_Mode_TypeDef;




#ifdef __APS__
#define gpio ((GPio *)SFRADR_GPIO1)
#else
extern GPio __gpio;
#define gpio (&__gpio)
#endif
#endif
