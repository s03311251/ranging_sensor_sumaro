/*
 * pl_softI2C.c
 *
 *  Created on: 8 Jan 2018
 *      Author: m2
 */

#include <pl_softI2C.h>

uint8_t pl_softI2C_delay_us = defaultDelay_us;
uint32_t pl_softI2C_timeout = defaultTimeout;

pl_softI2CDriver PI2CD1 = { { { GPIOB, 5, GPIOB, 4 }, { GPIOB, 10, GPIOA, 8 }, {
		GPIOA, 9, GPIOC, 7 }, { GPIOB, 6, GPIOA, 7 }, { GPIOB, 13, GPIOB, 14 }, {
		GPIOB, 15, GPIOB, 1 }, { GPIOB, 12, GPIOA, 11 }, { GPIOC, 5, GPIOC, 6 }, {
		GPIOC, 0, GPIOC, 1 }, { GPIOB, 0, GPIOA, 4 }, { GPIOA, 1, GPIOA, 0 }, {
		GPIOD, 2, GPIOC, 11 }, { GPIOC, 2, GPIOC, 3 }, { GPIOA, 15, GPIOC, 10 }, {
		GPIOC, 12, GPIOC, 4 }, { GPIOB, 2, GPIOA, 12 } }, 16 };

result_t pl_softI2C_alive[16];

void pl_softi2cMasterTransmitTimeout(pl_softI2CDriver *pi2cp, i2caddr_t addr,
		const uint8_t *txbuf, size_t txbytes, uint8_t rxbuf[][16], size_t rxbytes,
		systime_t timeout) {

	pl_softI2C_set_alive(pi2cp);
	pl_softI2C_timeout = timeout;

	pl_softI2C_llStartWait(pi2cp, (addr << 1));
	if (pl_softi2c_none_alive(pi2cp)) {
		return;
	}

	for (size_t i = 0; i < txbytes; i++) {
		pl_softI2C_write(pi2cp, txbuf[i]);
		if (pl_softi2c_none_alive(pi2cp)) {
			return;
		}
	}

	if (rxbytes > 0) {
		pl_softI2C_llRepeatedStart(pi2cp, (addr << 1) + 1);
		if (pl_softi2c_none_alive(pi2cp)) {
			return;
		}

		for (size_t i = 0; i < rxbytes - 1; i++) {
			pl_softI2C_read(pi2cp, *(rxbuf + i), true);
			if (pl_softi2c_none_alive(pi2cp)) {
				return;
			}
		}

		pl_softI2C_read(pi2cp, *(rxbuf + (rxbytes - 1)), false);
		if (pl_softi2c_none_alive(pi2cp)) {
			return;
		}
	}
	pl_softI2C_stop(pi2cp);
}

void pl_softi2cMasterReceiveTimeout(pl_softI2CDriver *pi2cp, i2caddr_t addr,
		uint8_t rxbuf[][16], size_t rxbytes, systime_t timeout) {

	for (int i = 0; i < pi2cp->driver_num; i++) {
		pl_softI2C_alive[i] = ack;
	}
	pl_softI2C_timeout = timeout;

	pl_softI2C_llStartWait(pi2cp, (addr << 1) + 1);
	if (pl_softi2c_none_alive(pi2cp)) {
		return;
	}

	for (size_t i = 0; i < rxbytes - 1; i++) {
		pl_softI2C_read(pi2cp, *(rxbuf + i), true);
		if (pl_softi2c_none_alive(pi2cp)) {
			return;
		}
	}

	pl_softI2C_read(pi2cp, *(rxbuf + (rxbytes - 1)), false);
	if (pl_softi2c_none_alive(pi2cp)) {
		return;
	}
	pl_softI2C_stop(pi2cp);
}

_Bool pl_softi2c_none_alive(pl_softI2CDriver *pi2cp) {
	for (int i = 0; i < pi2cp->driver_num; i++) {
		if (pl_softI2C_alive[i] == ack) {
			return false;
		}
	}
	return true;
}

void pl_softI2C_set_alive(const pl_softI2CDriver *pi2cp) {
	for (int i = 0; i < pi2cp->driver_num; i++) {
		pl_softI2C_alive[i] = ack;
	}
}

void pl_softI2C_setSdaLow(const pl_softI2CDriver *pi2cp) {
	for (int i = 0; i < pi2cp->driver_num; i++) {
			osalSysLock();
		palClearPad((pi2cp->SI2CD)[i].sdaPort, (pi2cp->SI2CD)[i].sdaPad);
			osalSysUnlock();
	}
}

void pl_softI2C_setSdaHigh(const pl_softI2CDriver *pi2cp) {
	for (int i = 0; i < pi2cp->driver_num; i++) {
			osalSysLock();
		palSetPad((pi2cp->SI2CD)[i].sdaPort, (pi2cp->SI2CD)[i].sdaPad);
			osalSysUnlock();
	}
}

void pl_softI2C_setSclLow(const pl_softI2CDriver *pi2cp) {
	for (int i = 0; i < pi2cp->driver_num; i++) {
			osalSysLock();
		palClearPad((pi2cp->SI2CD)[i].sclPort, (pi2cp->SI2CD)[i].sclPad);
			osalSysUnlock();
	}
}

void pl_softI2C_setSclHigh(const pl_softI2CDriver *pi2cp) {
	for (int i = 0; i < pi2cp->driver_num; i++) {
			osalSysLock();
		palSetPad((pi2cp->SI2CD)[i].sclPort, (pi2cp->SI2CD)[i].sclPad);
			osalSysUnlock();
	}
}

uint16_t pl_softI2C_readSda(const pl_softI2CDriver *pi2cp) {
	uint16_t value = 0;

	for (int i = 0; i < pi2cp->driver_num; i++) {
			osalSysLock();
		value |= (palReadPad((pi2cp->SI2CD)[i].sdaPort,
				(pi2cp->SI2CD)[i].sdaPad)) << i;
			osalSysUnlock();
	}

	return value;
 }

uint16_t pl_softI2C_readScl(const pl_softI2CDriver *pi2cp) {
	uint16_t value = 0;

	for (int i = 0; i < pi2cp->driver_num; i++) {
			osalSysLock();
		value |= (palReadPad((pi2cp->SI2CD)[i].sclPort,
				(pi2cp->SI2CD)[i].sclPad)) << i;
			osalSysUnlock();
	}

	return value;
}

//void pl_softI2C_alive_setSdaLow(const pl_softI2CDriver *pi2cp) {
//	for (int i = 0; i < pi2cp->driver_num; i++) {
//		if (pl_softI2C_alive[i] == ack) {
//			osalSysLock();
//			palClearPad((pi2cp->SI2CD)[i].sdaPort, (pi2cp->SI2CD)[i].sdaPad);
//			osalSysUnlock();
//		}
//	}
//}
//
//void pl_softI2C_alive_setSdaHigh(const pl_softI2CDriver *pi2cp) {
//	for (int i = 0; i < pi2cp->driver_num; i++) {
//		if (pl_softI2C_alive[i] == ack) {
//			osalSysLock();
//			palSetPad((pi2cp->SI2CD)[i].sdaPort, (pi2cp->SI2CD)[i].sdaPad);
//			osalSysUnlock();
//		}
//	}
//}
//
//void pl_softI2C_alive_setSclLow(const pl_softI2CDriver *pi2cp) {
//	for (int i = 0; i < pi2cp->driver_num; i++) {
//		if (pl_softI2C_alive[i] == ack) {
//			osalSysLock();
//			palClearPad((pi2cp->SI2CD)[i].sclPort, (pi2cp->SI2CD)[i].sclPad);
//			osalSysUnlock();
//		}
//	}
//}
//
//void pl_softI2C_alive_setSclHigh(const pl_softI2CDriver *pi2cp) {
//	for (int i = 0; i < pi2cp->driver_num; i++) {
//		if (pl_softI2C_alive[i] == ack) {
//			osalSysLock();
//			palSetPad((pi2cp->SI2CD)[i].sclPort, (pi2cp->SI2CD)[i].sclPad);
//			osalSysUnlock();
//		}
//	}
//}
//
//uint16_t pl_softI2C_alive_readSda(const pl_softI2CDriver *pi2cp) {
//	uint16_t value = 0;
//
//	for (int i = 0; i < pi2cp->driver_num; i++) {
//		if (pl_softI2C_alive[i] == ack) {
//			osalSysLock();
//			value |= (palReadPad((pi2cp->SI2CD)[i].sdaPort,
//					(pi2cp->SI2CD)[i].sdaPad)) << i;
//			osalSysUnlock();
//		}
//	}
//
//	return value;
// }
//
//uint16_t pl_softI2C_alive_readScl(const pl_softI2CDriver *pi2cp) {
//	uint16_t value = 0;
//
//	for (int i = 0; i < pi2cp->driver_num; i++) {
//		if (pl_softI2C_alive[i] == ack) {
//			osalSysLock();
//			value |= (palReadPad((pi2cp->SI2CD)[i].sclPort,
//					(pi2cp->SI2CD)[i].sclPad)) << i;
//			osalSysUnlock();
//		}
//	}
//
//	return value;
//}

//void pl_softI2C_alive_readScl(const pl_softI2CDriver *pi2cp, _Bool value[]) {
//	for (int i = 0; i < pi2cp->driver_num; i++) {
//		value[i] = 0;
//	}
//
//	for (int i = 0; i < pi2cp->driver_num; i++) {
//		if (pl_softI2C_alive[i] == ack) {
//			osalSysLock();
//			value[i] = (palReadPad((pi2cp->SI2CD)[i].sclPort,
//					(pi2cp->SI2CD)[i].sclPad));
//			osalSysUnlock();
//		}
//	}
//}

void pl_softI2C_stop(const pl_softI2CDriver *pi2cp) {
	// Force SDA low
	pl_softI2C_setSdaLow(pi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Release SCL
	pl_softI2C_setSclHigh(pi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Release SDA
	pl_softI2C_setSdaHigh(pi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);
}

void pl_softI2C_stop_if_not_alive(const pl_softI2CDriver *pi2cp) {
	// Force SDA low
	for (int i = 0; i < pi2cp->driver_num; i++) {
		osalSysLock();
		if (pl_softI2C_alive[i] != ack) {
			palClearPad((pi2cp->SI2CD)[i].sdaPort, (pi2cp->SI2CD)[i].sdaPad);
		}
		osalSysUnlock();
	}
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Release SCL
	for (int i = 0; i < pi2cp->driver_num; i++) {
		osalSysLock();
		if (pl_softI2C_alive[i] != ack) {
			palSetPad((pi2cp->SI2CD)[i].sclPort, (pi2cp->SI2CD)[i].sclPad);
		}
		osalSysUnlock();
	}
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Release SDA
	for (int i = 0; i < pi2cp->driver_num; i++) {
		osalSysLock();
		if (pl_softI2C_alive[i] != ack) {
			palSetPad((pi2cp->SI2CD)[i].sdaPort, (pi2cp->SI2CD)[i].sdaPad);
		}
		osalSysUnlock();
	}
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);
}

void pl_softI2C_llRepeatedStart(const pl_softI2CDriver *pi2cp, uint8_t rawAddr) {
	// Force SCL low
	pl_softI2C_setSclLow(pi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Release SDA
	pl_softI2C_setSdaHigh(pi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Release SCL
	pl_softI2C_setSclHigh(pi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Force SDA low
	pl_softI2C_setSdaLow(pi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	pl_softI2C_write(pi2cp, rawAddr);
}

void pl_softI2C_llStartWait(const pl_softI2CDriver *pi2cp, uint8_t rawAddr) {
	systime_t timeout_start = chVTGetSystemTime();

	while (chVTTimeElapsedSinceX(timeout_start) <= pl_softI2C_timeout) {
		// Force SDA low
		pl_softI2C_setSdaLow(pi2cp);
		gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

		pl_softI2C_write(pi2cp, rawAddr);
		pl_softI2C_stop_if_not_alive(pi2cp);

//		for (int i = 0; i < pi2cp->driver_num; i++) {
//			switch (pl_softI2C_alive[i]) {
//			case ack:
//				break;
//			case nack:
//				softI2C_stop((pi2cp->SI2CD) + i);
//				break;
//			default:
//				softI2C_stop((pi2cp->SI2CD) + i);
//			}
//		}
		return;
	}
}

void pl_softI2C_write(const pl_softI2CDriver *pi2cp, uint8_t data) {
	systime_t timeout_start = chVTGetSystemTime();
	for (uint8_t i = 8; i; --i) {
		// Force SCL low
		pl_softI2C_setSclLow(pi2cp);

		if (data & 0x80) { // MSB
			// Release SDA
			pl_softI2C_setSdaHigh(pi2cp);
		} else {
			// Force SDA low
			pl_softI2C_setSdaLow(pi2cp);
		}
		gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

		// Release SCL
		pl_softI2C_setSclHigh(pi2cp);
		gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

		data <<= 1;

		if (chVTTimeElapsedSinceX(timeout_start) > pl_softI2C_timeout) {
			pl_softI2C_stop(pi2cp); // Reset bus
			for (int i = 0; i < pi2cp->driver_num; i++) {
				pl_softI2C_alive[i] = timedOut;
			}
		}
	}

	// Get ACK
	// Force SCL low
	pl_softI2C_setSclLow(pi2cp);

	// Release SDA
	pl_softI2C_setSdaHigh(pi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Release SCL
	pl_softI2C_setSclHigh(pi2cp);

	// Wait for SCL to be set high (in case wait states are inserted)
	uint16_t scl_value = pl_softI2C_readScl(pi2cp);
	while (scl_value != 0xFF) {
		if (chVTTimeElapsedSinceX(timeout_start) > pl_softI2C_timeout) {
			for (int i = 0; i < pi2cp->driver_num; i++) {
				if (((scl_value >> i) & 1) == 0) {
					softI2C_stop((pi2cp->SI2CD) + i); // Reset bus
					pl_softI2C_alive[i] = timedOut;
				}
			}
			break;
		}
	}

	uint16_t sda_value = pl_softI2C_readSda(pi2cp);
	for (int i = 0; i < pi2cp->driver_num; i++) {
		if (pl_softI2C_alive[i] != timedOut) {
			if (((sda_value >> i) & 1)) {
				pl_softI2C_alive[i] = nack;
			} else {
				pl_softI2C_alive[i] = ack;
			}
		}
	}

	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Keep SCL low between bytes
	pl_softI2C_setSclLow(pi2cp);
}

void pl_softI2C_read(const pl_softI2CDriver *pi2cp, uint8_t data[16],
_Bool sendAck) {

	for (uint8_t i = 8; i; --i) {
		data[i] = 0;
	}
	systime_t timeout_start = chVTGetSystemTime();

	for (uint8_t i = 8; i; --i) {
		data[i] <<= 1;

		// Force SCL low
		pl_softI2C_setSclLow(pi2cp);

		// Release SDA (from previous ACK)
		pl_softI2C_setSdaHigh(pi2cp);
		gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

		// Release SCL
		pl_softI2C_setSclHigh(pi2cp);
		gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

		// Read clock stretch
		uint16_t scl_value = pl_softI2C_readScl(pi2cp);
		while (scl_value != 0xFF) {
			if (chVTTimeElapsedSinceX(timeout_start) > pl_softI2C_timeout) {
				for (int i = 0; i < pi2cp->driver_num; i++) {
					if (((scl_value >> i) & 1) == 0) {
						softI2C_stop((pi2cp->SI2CD) + i); // Reset bus
						pl_softI2C_alive[i] = timedOut;
					}
				}
				break;
			}
		}

		uint16_t sda_value = pl_softI2C_readSda(pi2cp);
		for (int j = 0; j < pi2cp->driver_num; j++) {
//			if (softI2C_readSda((pi2cp->SI2CD) + j)) {
//				data[j] |= 1;
//			}
			data[j] |= (sda_value >> j) << i;
		}

	}

	// Put ACK/NACK

	// Force SCL low
	pl_softI2C_setSclLow(pi2cp);
	if (sendAck) {
		// Force SDA low
		pl_softI2C_setSdaLow(pi2cp);
	} else {
		// Release SDA
		pl_softI2C_setSdaHigh(pi2cp);
	}

	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Release SCL
	pl_softI2C_setSclHigh(pi2cp);

	// Wait for SCL to return high
	uint16_t scl_value = pl_softI2C_readScl(pi2cp);
	while (scl_value != 0xFF) {
		if (chVTTimeElapsedSinceX(timeout_start) > pl_softI2C_timeout) {
			for (int i = 0; i < pi2cp->driver_num; i++) {
				if (((scl_value >> i) & 1) == 0) {
					softI2C_stop((pi2cp->SI2CD) + i); // Reset bus
					pl_softI2C_alive[i] = timedOut;
				}
			}
			break;
		}
	}

	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Keep SCL low between bytes
	pl_softI2C_setSclLow(pi2cp);
}

