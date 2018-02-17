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

#include "VL53L0X.h"

#include "chprintf.h"

#define LINE_ARD_D14                PAL_LINE(GPIOB, 9U)
#define LINE_ARD_D15                PAL_LINE(GPIOB, 8U)

BaseSequentialStream* chp = (BaseSequentialStream*) &SD2;

static const I2CConfig i2ccfg = { OPMODE_I2C, 100000, STD_DUTY_CYCLE };
//static const I2CConfig i2ccfg = { OPMODE_I2C, 400000, FAST_DUTY_CYCLE_2 };

/*
 * Green LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

	(void) arg;
	chRegSetThreadName("blinker");
	while (true) {
		palClearPad(GPIOA, GPIOA_LED_GREEN);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOA, GPIOA_LED_GREEN);
		chThdSleepMilliseconds(500);
	}
}

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

	sdStart(&SD2, NULL);
	i2cStart(&I2CD1, &i2ccfg);

	/*
	 * Creates the blinker thread.
	 */
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1,
			NULL);

	/* Configuring I2C related PINs
	 * I2C1: SDA: PB9  , SCL: PB8
	 * I2C2: SDA: PB3  , SCL: PB10
	 * I2C3: SDA: PC9  , SCL: PA8 */

	palSetLineMode(LINE_ARD_D15,
			PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
	palSetLineMode(LINE_ARD_D14,
			PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);

	for (int i = 0; i < VL53L0X_COUNT; i++) {
		palSetPadMode(VB[i].xshut_port, VB[i].xshut_pad, PAL_MODE_OUTPUT_OPENDRAIN);
		palClearPad(VB[i].xshut_port, VB[i].xshut_pad);
	}

	VL53L0X_setTimeout(50);
	for (int i = 0; i < VL53L0X_COUNT; i++) {
		palSetPad(VB[i].xshut_port, VB[i].xshut_pad);
		// HW standby, I don't know how long it should be
		chThdSleepMilliseconds(2);
		VL53L0X_setAddress(VB[i]);
		VL53L0X_init(VB[i], true);
//		VL53L0X_setProfile(VB[i], VL53L0X_HighSpeed);
		VL53L0X_startContinuous(VB[i]);
	}

	while (true) {
//		for (int i = 0; i < VL53L0X_COUNT; i++) {
//			chprintf(chp, "%2d: %5d ", i,
//					VL53L0X_readRangeContinuousMillimeters(VB[i]));
//		}
//		chprintf(chp, "\r\n");

		VL53L0X_readRangeContinuousMillimeters_loop(VB, VL53L0X_COUNT, 1000);
	}
}
