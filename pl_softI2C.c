/*
 * pl_softI2C.c
 *
 *  Created on: 8 Jan 2018
 *      Author: m2
 */

#include <pl_softI2C.h>

uint8_t pl_softI2C_delay_us = defaultDelay_us;
uint32_t pl_softI2C_timeout = defaultTimeout;

pl_softI2CDriver PI2CD1 = { { { GPIOB, 5, GPIOB, 4 }, { GPIOB, 10, GPIOA, 8 } },
		2 };
/*
void softi2cMasterTransmitTimeout(softI2CDriver *si2cp,
		i2caddr_t addr, // 7-bit address
		const uint8_t *txbuf, size_t txbytes, uint8_t *rxbuf, size_t rxbytes,
		systime_t timeout) {
	pl_softI2C_timeout = timeout;

	if (softI2C_llStartWait(si2cp, addr << 1) != ack) {
		return;
	}

	for (size_t i = 0; i < txbytes; i++) {
		if (softI2C_write(si2cp, txbuf[i]) != ack) {
			return;
		}
	}

	if (rxbytes > 0) {
		if (softI2C_llRepeatedStart(si2cp, (addr << 1) + 1) != ack) {
			return;
		}

		for (size_t i = 0; i < rxbytes - 1; i++) {
			if (softI2C_read(si2cp, &rxbuf[i], true) != ack) {
				return;
			}
		}

		if (softI2C_read(si2cp, &rxbuf[rxbytes - 1], false) != ack) {
			return;
		}
	}
	softI2C_stop(si2cp);
}

void softi2cMasterReceiveTimeout(softI2CDriver *si2cp, i2caddr_t addr, // 7-bit address
		uint8_t *rxbuf, size_t rxbytes, systime_t timeout) {
	pl_softI2C_timeout = timeout;

	if (softI2C_llStartWait(si2cp, (addr << 1) + 1) != ack) {
		return;
	}

	for (size_t i = 0; i < rxbytes - 1; i++) {
		if (softI2C_read(si2cp, &rxbuf[i], true) != ack) {
			return;
		}
	}

	if (softI2C_read(si2cp, &rxbuf[rxbytes - 1], false) != ack) {
		return;
	}
	softI2C_stop(si2cp);
}
 */

_Bool pl_softI2C_all_one(const pl_softI2CDriver *psi2cp, _Bool value[]) {
	for (int i = 0; i < psi2cp->driver_num; i++) {
		if (value[i] == false) {
			return false;
		}
	}
	return true;
}

void pl_softI2C_setSdaLow(const pl_softI2CDriver *psi2cp) {
	osalSysLock();
	for (int i = 0; i < psi2cp->driver_num; i++) {
		palClearPad((psi2cp->SI2CD)[i].sdaPort, (psi2cp->SI2CD)[i].sdaPad);
	}
	osalSysUnlock();
}

void pl_softI2C_setSdaHigh(const pl_softI2CDriver *psi2cp) {
	osalSysLock();
	for (int i = 0; i < psi2cp->driver_num; i++) {
		palSetPad((psi2cp->SI2CD)[i].sdaPort, (psi2cp->SI2CD)[i].sdaPad);
	}
	osalSysUnlock();
}

void pl_softI2C_setSclLow(const pl_softI2CDriver *psi2cp) {
	osalSysLock();
	for (int i = 0; i < psi2cp->driver_num; i++) {
		palClearPad((psi2cp->SI2CD)[i].sdaPort, (psi2cp->SI2CD)[i].sdaPad);
	}
	osalSysUnlock();
}

void pl_softI2C_setSclHigh(const pl_softI2CDriver *psi2cp) {
	osalSysLock();
	for (int i = 0; i < psi2cp->driver_num; i++) {
		palSetPad((psi2cp->SI2CD)[i].sclPort, (psi2cp->SI2CD)[i].sclPad);
	}
	osalSysUnlock();
}

void pl_softI2C_readSda(const pl_softI2CDriver *psi2cp, _Bool value[]) {
	osalSysLock();
	for (int i = 0; i < psi2cp->driver_num; i++) {
		value[i] = palReadPad((psi2cp->SI2CD)[i].sdaPort,
				(psi2cp->SI2CD)[i].sdaPad);
	}
	osalSysUnlock();
 }

void pl_softI2C_readScl(const pl_softI2CDriver *psi2cp, _Bool value[]) {
	osalSysLock();
	for (int i = 0; i < psi2cp->driver_num; i++) {
		value[i] = palReadPad((psi2cp->SI2CD)[i].sclPort,
				(psi2cp->SI2CD)[i].sclPad);
	}
	osalSysUnlock();
 }

void pl_softI2C_stop(const pl_softI2CDriver *psi2cp) {
	// Force SDA low
	pl_softI2C_setSdaLow(psi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Release SCL
	pl_softI2C_setSclHigh(psi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Release SDA
	pl_softI2C_setSdaHigh(psi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);
}

result_t pl_softI2C_llRepeatedStart(const pl_softI2CDriver *psi2cp,
		uint8_t rawAddr) {
	// Force SCL low
	pl_softI2C_setSclLow(psi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Release SDA
	pl_softI2C_setSdaHigh(psi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Release SCL
	pl_softI2C_setSclHigh(psi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Force SDA low
	pl_softI2C_setSdaLow(psi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	return pl_softI2C_write(psi2cp, rawAddr);
}

result_t pl_softI2C_llStartWait(const pl_softI2CDriver *psi2cp, uint8_t rawAddr) {
	systime_t timeout_start = chVTGetSystemTime();

	while (chVTTimeElapsedSinceX(timeout_start) <= pl_softI2C_timeout) {
		// Force SDA low
		pl_softI2C_setSdaLow(psi2cp);
		gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

		switch (pl_softI2C_write(psi2cp, rawAddr)) {
		case ack:
			return ack;
		case nack:
			// Arduino SoftWire stop() twice here WTF?
			softI2C_stop(si2cp);
			return nack;
		default:
			// timeout, and anything else we don't know about
			softI2C_stop(si2cp);
			return timedOut;
		}
	}
	return timedOut;
}

result_t pl_softI2C_write(const pl_softI2CDriver *psi2cp, uint8_t data) {
	systime_t timeout_start = chVTGetSystemTime();
	for (uint8_t i = 8; i; --i) {
		// Force SCL low
		pl_softI2C_setSclLow(psi2cp);

		if (data & 0x80) { // MSB
			// Release SDA
			pl_softI2C_setSdaHigh(psi2cp);
		} else {
			// Force SDA low
			pl_softI2C_setSdaLow(psi2cp);
		}
		gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

		// Release SCL
		pl_softI2C_setSclHigh(psi2cp);
		gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

		data <<= 1;

		if (chVTTimeElapsedSinceX(timeout_start) > pl_softI2C_timeout) {
			pl_softI2C_stop(psi2cp); // Reset bus
			return timedOut;
		}
	}

	// Get ACK
	// Force SCL low
	pl_softI2C_setSclLow(psi2cp);

	// Release SDA
	pl_softI2C_setSdaHigh(psi2cp);
	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Release SCL
	pl_softI2C_setSclHigh(psi2cp);

	// Wait for SCL to be set high (in case wait states are inserted)
	_Bool scl_value[psi2cp.driver_num];
	pl_softI2C_readScl(psi2cp, &scl_value);

	while (pl_softI2C_all_one(psi2cp, scl_value) == false) {
		if (chVTTimeElapsedSinceX(timeout_start) > pl_softI2C_timeout) {
			pl_softI2C_stop(psi2cp); // Reset bus
			return timedOut;
		}
	}

	result_t res = (pl_softI2C_readSda(psi2cp) == 0 ? ack : nack);

	gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Keep SCL low between bytes
	pl_softI2C_setSclLow(psi2cp);

	return res;
}
/*
result_t softI2C_read(const softI2CDriver *si2cp, uint8_t *data, _Bool sendAck) {
	*data = 0;
	//systime_t timeout_start = chVTGetSystemTime();
	uint32_t timeout_start = chVTGetSystemTime();

	for (uint8_t i = 8; i; --i) {
		*data <<= 1;

		// Force SCL low
		softI2C_setSclLow(si2cp);

		// Release SDA (from previous ACK)
		softI2C_setSdaHigh(si2cp);
 gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

		// Release SCL
		softI2C_setSclHigh(si2cp);
 gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

		// Read clock stretch
		while (softI2C_readScl(si2cp) == 0)
 if (chVTTimeElapsedSinceX(timeout_start) > pl_softI2C_timeout) {
				softI2C_stop(si2cp); // Reset bus
				return timedOut;
			}

		if (softI2C_readSda(si2cp))
			*data |= 1;
	}

	// Put ACK/NACK

	// Force SCL low
	softI2C_setSclLow(si2cp);
	if (sendAck) {
		// Force SDA low
		softI2C_setSdaLow(si2cp);
	} else {
		// Release SDA
		softI2C_setSdaHigh(si2cp);
	}

 gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Release SCL
	softI2C_setSclHigh(si2cp);

	// Wait for SCL to return high
	while (softI2C_readScl(si2cp) == 0) {
 if (chVTTimeElapsedSinceX(timeout_start) > pl_softI2C_timeout) {
			softI2C_stop(si2cp); // Reset bus
			return timedOut;
		}
	}

 gptPolledDelay(&GPTD4, pl_softI2C_delay_us);

	// Keep SCL low between bytes
	softI2C_setSclLow(si2cp);

	return ack;
}



 */
