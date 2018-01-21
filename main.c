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
#include "chprintf.h"

#include "VL53L0X.h"
#include "softI2C.h"
#include "pl_softI2C.h"

BaseSequentialStream* chp = (BaseSequentialStream*) &SD2;

#define LINE_ARD_D14                PAL_LINE(GPIOB, 9U)
#define LINE_ARD_D15                PAL_LINE(GPIOB, 8U)

//_Bool test = false;

//static void gpt4cb(GPTDriver *gptp) {
//
//	(void) gptp;
//
//	if (test == false)
//		palSetPad(GPIOA, GPIOA_LED_GREEN);
//	else
//		palClearPad(GPIOA, GPIOA_LED_GREEN);
//	test = !test;
//
//	chSysLockFromISR();
//	gptStartOneShotI(&GPTD4, 1);
//	chSysUnlockFromISR();
//}




static const I2CConfig i2ccfg = { OPMODE_I2C, 100000, STD_DUTY_CYCLE };
static const GPTConfig gpt4cfg = { 500000, NULL, 0, 0 };

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
	gptStart(&GPTD4, &gpt4cfg);

	/*
	 * Creates the blinker thread.
	 */
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1,
	NULL);

	/* Configuring I2C related PINs */

	palSetLineMode(LINE_ARD_D15,
			PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
	palSetLineMode(LINE_ARD_D14,
			PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);

	for (int i = 0; i < 16; i++) {
		palSetPadMode(PI2CD1.SI2CD[i].sdaPort, PI2CD1.SI2CD[i].sdaPad,
				PAL_MODE_OUTPUT_OPENDRAIN);
		palSetPadMode(PI2CD1.SI2CD[i].sclPort, PI2CD1.SI2CD[i].sclPad,
				PAL_MODE_OUTPUT_OPENDRAIN);
	}
//	palSetPadMode(SI2CD1.sdaPort, SI2CD1.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD1.sclPort, SI2CD1.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD2.sdaPort, SI2CD2.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD2.sclPort, SI2CD2.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD3.sdaPort, SI2CD3.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD3.sclPort, SI2CD3.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD4.sdaPort, SI2CD4.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD4.sclPort, SI2CD4.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD5.sdaPort, SI2CD5.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD5.sclPort, SI2CD5.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD6.sdaPort, SI2CD6.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD6.sclPort, SI2CD6.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD7.sdaPort, SI2CD7.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD7.sclPort, SI2CD7.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD8.sdaPort, SI2CD8.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD8.sclPort, SI2CD8.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD9.sdaPort, SI2CD9.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD9.sclPort, SI2CD9.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD10.sdaPort, SI2CD10.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD10.sclPort, SI2CD10.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD11.sdaPort, SI2CD11.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD11.sclPort, SI2CD11.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD12.sdaPort, SI2CD12.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD12.sclPort, SI2CD12.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD13.sdaPort, SI2CD13.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD13.sclPort, SI2CD13.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD14.sdaPort, SI2CD14.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD14.sclPort, SI2CD14.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD15.sdaPort, SI2CD15.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD15.sclPort, SI2CD15.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD16.sdaPort, SI2CD16.sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(SI2CD16.sclPort, SI2CD16.sclPad, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(GPIOB, 5U, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(GPIOB, 4U, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(GPIOB, 10U, PAL_MODE_OUTPUT_OPENDRAIN);
//	palSetPadMode(GPIOA, 8U, PAL_MODE_OUTPUT_OPENDRAIN);

//	VL53L0X_init();
//	VL53L0X_setTimeout(500);
//	VL53L0X_startContinuous();

	while (true) {

// Testing VL53L0X
		
//		chprintf(chp, "\rabc:%4d", VL53L0X_readRangeContinuousMillimeters());
//		if (VL53L0X_timeoutOccurred()) {
//			chprintf(chp, "TIMEOUT");
//		}

// Testing timer
		
//		gptStartContinuous(&GPTD4, 1);
//		gptStartOneShotI(&GPTD4, 1);

//		palSetPad(GPIOA, GPIOA_LED_GREEN);
//		gptPolledDelay(&GPTD4, 1);
//		palClearPad(GPIOA, GPIOA_LED_GREEN);
//		gptPolledDelay(&GPTD4, 1);

// Testing softI2C

//		uint8_t txbuf[3] = { 'a', 'b', 0x10 };
//		softi2cMasterTransmitTimeout(&SI2CD1, 0x04, txbuf, 3, NULL, 0,
//		TIME_INFINITE);

//		uint8_t rxbuf[6] = { 'U', 'f', 'a', 'i', 'l', ' ', };
//		softi2cMasterReceiveTimeout(&SI2CD1, 0x10, rxbuf, 6, TIME_INFINITE);
//		for (int i = 0; i < 6; i++)
//			sdPut(&SD2, rxbuf[i]);

//		uint8_t txbuf[2] = { 'a', 'A' };
//		uint8_t rxbuf[2] = { 'z', 'Z' };
//		softi2cMasterTransmitTimeout(&SI2CD1, 0x10, txbuf, 2, rxbuf, 2,
//		TIME_INFINITE);
//		for (int i = 0; i < 2; i++)
//			sdPut(&SD2, rxbuf[i]);

//	i2cMasterTransmitTimeout(&I2CD1, 0x10, txbuf, 2, rxbuf, 2,
//		TIME_INFINITE);
//		for (int i = 0; i < 2; i++)
//			sdPut(&SD2, rxbuf[i]);

//		uint8_t txbuf2[2] = { 'c', 'C' };
//		uint8_t rxbuf2[2] = { 'y', 'Y' };
//		softi2cMasterTransmitTimeout(&SI2CD2, 0x10, txbuf2, 2, rxbuf2, 2,
//		TIME_INFINITE);
//		for (int i = 0; i < 2; i++)
//			sdPut(&SD2, rxbuf[i]);

// Testing parallel softI2C
		pl_softI2C_llStartWait(&PI2CD1, 0x10 << 1);
//		pl_softI2C_set_alive(&PI2CD1);
//		
//		_Bool scl_value[16];
//		
//		pl_softI2C_setSclHigh(&PI2CD1);
//		chThdSleepMilliseconds(1);
//		pl_softI2C_readScl(&PI2CD1, scl_value);
//		for (int i = 0; i < 16; i++) {
//			chprintf(chp, "%d ", scl_value[i]);
//		}
//		chprintf(chp, "\r\n");
//		
//		pl_softI2C_setSclLow(&PI2CD1);
//		chThdSleepMilliseconds(1);
//		pl_softI2C_readScl(&PI2CD1, scl_value);
//		for (int i = 0; i < 16; i++) {
//			chprintf(chp, "%d ", scl_value[i]);
//		}
//		chprintf(chp, "\r\n");

//		softI2C_write(&SI2CD1, 0x10 << 1);
//		softI2C_stop(&SI2CD1);

		chThdSleepMilliseconds(100);

	}
}
