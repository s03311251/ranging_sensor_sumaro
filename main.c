/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "ch_test.h"

//#include "VL53L0X.h"
#include "softI2C.h"

#define LINE_ARD_D14                PAL_LINE(GPIOB, 9U)
#define LINE_ARD_D15                PAL_LINE(GPIOB, 8U)

static const I2CConfig i2ccfg = { OPMODE_I2C, 100000, STD_DUTY_CYCLE, };

/*
 * Green LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palClearPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(500);
  }
}

/*
 * Don't know do what good

static THD_WORKING_AREA(waThread2, 128);
static THD_FUNCTION(Thread2, arg) {

	(void) arg;
	chRegSetThreadName("blinker");
	while (true) {
		palClearPad(GPIOA, GPIOA_LED_GREEN);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOA, GPIOA_LED_GREEN);
		chThdSleepMilliseconds(500);
	}
}
 */

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 2 using the driver default configuration.
   */
  sdStart(&SD2, NULL);

	/*
	 * Creates the blinker thread.
	 */
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1,
			NULL);
	//chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO + 1, Thread2,
	//		NULL);

	/* Configuring I2C related PINs */
	palSetLineMode(LINE_ARD_D15,
			PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN
					| PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
	palSetLineMode(LINE_ARD_D14,
			PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN
					| PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);

	i2cStart(&I2CD1, &i2ccfg);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    if (!palReadPad(GPIOC, GPIOC_BUTTON))
      test_execute((BaseSequentialStream *)&SD2);
		uint8_t txbuf[3] = { 'a', 'b', 0x10 };
		i2cMasterTransmitTimeout(&I2CD1, 0x04, txbuf, 3, NULL, 0,
				TIME_INFINITE);
		chThdSleepMilliseconds(100);
  }
}
