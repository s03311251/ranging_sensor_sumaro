/*
 * softI2C.c
 *
 *  Created on: 31 Dec 2017
 *      Author: m2
 */

#include "softI2C.h"

uint8_t softI2C_delay_us = defaultDelay_us;
uint32_t softI2C_timeout = defaultTimeout;

softI2CDriver SI2CD1 = { GPIOB, 5, GPIOB, 4 };
softI2CDriver SI2CD2 = { GPIOB, 10, GPIOA, 8 };
//	softI2CDriver SI2CD3 = { GPIOA, 9, GPIOC, 7 };
//	softI2CDriver SI2CD4 = { GPIOB, 6, GPIOA, 7 };
//	softI2CDriver SI2CD5 = { GPIOB, 13, GPIOB, 14 };
//	softI2CDriver SI2CD6 = { GPIOB, 15, GPIOB, 1 };
//	softI2CDriver SI2CD7 = { GPIOB, 12, GPIOA, 11 };
//	softI2CDriver SI2CD8 = { GPIOC, 5, GPIOC, 6 };
//	softI2CDriver SI2CD9 = { GPIOC, 0, GPIOC, 1 };
//	softI2CDriver SI2CD10 = { GPIOB, 0, GPIOA, 4 };
//	softI2CDriver SI2CD11 = { GPIOA, 1, GPIOA, 0 };
//	softI2CDriver SI2CD12 = { GPIOD, 2, GPIOC, 11 };
//	softI2CDriver SI2CD13 = { GPIOC, 2, GPIOC, 3 };
//	softI2CDriver SI2CD14 = { GPIOA, 15, GPIOC, 10 };
//	softI2CDriver SI2CD15 = { GPIOC, 12, GPIOC, 4 };
//	softI2CDriver SI2CD16 = { GPIOB, 2, GPIOA, 12 };

/*
 * @retval MSG_OK       ack
 * @retval MSG_RESET    nack
 * @retval MSG_TIMEOUT  timeOut
 */
msg_t softi2cMasterTransmitTimeout(softI2CDriver *si2cp,
		i2caddr_t addr, // 7-bit address
		const uint8_t *txbuf, size_t txbytes, uint8_t *rxbuf, size_t rxbytes,
		systime_t timeout) {
	softI2C_timeout = timeout;

	result_t current_msg = softI2C_llStartWait(si2cp, addr << 1);
	if (current_msg == nack) {
		return MSG_RESET ;
	} else if (current_msg == timedOut) {
		return MSG_TIMEOUT ;
	}

	for (size_t i = 0; i < txbytes; i++) {
		current_msg = softI2C_write(si2cp, txbuf[i]);
		if (current_msg == nack) {
			return MSG_RESET ;
		} else if (current_msg == timedOut) {
			return MSG_TIMEOUT ;
		}
	}

	if (rxbytes > 0) {
		current_msg = softI2C_llRepeatedStart(si2cp, (addr << 1) + 1);
		if (current_msg == nack) {
			return MSG_RESET ;
		} else if (current_msg == timedOut) {
			return MSG_TIMEOUT ;
		}

		for (size_t i = 0; i < rxbytes - 1; i++) {
			current_msg = softI2C_read(si2cp, &rxbuf[i], true);
			if (current_msg == nack) {
				return MSG_RESET ;
			} else if (current_msg == timedOut) {
				return MSG_TIMEOUT ;
			}
		}

		softI2C_read(si2cp, &rxbuf[rxbytes - 1], false);
		if (current_msg == nack) {
			return MSG_RESET ;
		} else if (current_msg == timedOut) {
			return MSG_TIMEOUT ;
		}
	}

	softI2C_stop(si2cp);
	return MSG_OK ;
}

msg_t softi2cMasterReceiveTimeout(softI2CDriver *si2cp, i2caddr_t addr, // 7-bit address
		uint8_t *rxbuf, size_t rxbytes, systime_t timeout) {
	softI2C_timeout = timeout;

	result_t current_msg = softI2C_llStartWait(si2cp, (addr << 1) + 1);
	if (current_msg == nack) {
		return MSG_RESET ;
	} else if (current_msg == timedOut) {
		return MSG_TIMEOUT ;
	}

	for (size_t i = 0; i < rxbytes - 1; i++) {
		current_msg = softI2C_read(si2cp, &rxbuf[i], true);
		if (current_msg == nack) {
			return MSG_RESET ;
		} else if (current_msg == timedOut) {
			return MSG_TIMEOUT ;
		}
	}

	current_msg = softI2C_read(si2cp, &rxbuf[rxbytes - 1], false);
	if (current_msg == nack) {
		return MSG_RESET ;
	} else if (current_msg == timedOut) {
		return MSG_TIMEOUT ;
	}

	softI2C_stop(si2cp);
	return MSG_OK ;
}

 // Force SDA low
void softI2C_setSdaLow(const softI2CDriver *si2cp) {
	osalSysLock();
	palClearPad(si2cp->sdaPort, si2cp->sdaPad);
	osalSysUnlock();
}

 // Release SDA to float high
void softI2C_setSdaHigh(const softI2CDriver *si2cp) {
	osalSysLock();
	palSetPad(si2cp->sdaPort, si2cp->sdaPad);
	osalSysUnlock();
}

 // Force SCL low
void softI2C_setSclLow(const softI2CDriver *si2cp) {
	osalSysLock();
	palClearPad(si2cp->sclPort, si2cp->sclPad);
	osalSysUnlock();
}

 // Release SCL to float high
void softI2C_setSclHigh(const softI2CDriver *si2cp) {
	osalSysLock();
	palSetPad(si2cp->sclPort, si2cp->sclPad);
	osalSysUnlock();
}

 // Read SDA (for data read)
_Bool softI2C_readSda(const softI2CDriver *si2cp) {
	osalSysLock();
	uint8_t value = palReadPad(si2cp->sdaPort, si2cp->sdaPad);
	osalSysUnlock();
	return value;
 }

 // Read SCL (to detect clock-stretching)
_Bool softI2C_readScl(const softI2CDriver *si2cp) {
	osalSysLock();
	uint8_t value = palReadPad(si2cp->sclPort, si2cp->sclPad);
	osalSysUnlock();
	return value;
 }

/*
 // For testing the CRC-8 calculator may be useful:
 // http://smbus.org/faq/crc8Applet.htm
 uint8_t SoftWire::crc8_update(uint8_t crc, uint8_t data)
 {
 const uint16_t polynomial = 0x107;
 crc ^= data;
 for (uint8_t i = 8; i; --i) {
 if (crc & 0x80)
 crc = (uint16_t(crc) << 1) ^ polynomial;
 else
 crc <<= 1;
 }

 return crc;
 }


 SoftWire::SoftWire(uint8_t sda, uint8_t scl) :
 _sda(sda),
 _scl(scl),
 _inputMode(INPUT), // Pullups diabled by default
 _delay_us(defaultDelay_us),
 _timeout_ms(defaultTimeout_ms),
 _setSdaLow(setSdaLow),
 _setSdaHigh(setSdaHigh),
 _setSclLow(setSclLow),
 _setSclHigh(setSclHigh),
 _readSda(readSda),
 _readScl(readScl)
 {
 ;
 }
 */
void softI2C_begin(const softI2CDriver *si2cp) {
	/*
	 // Release SDA and SCL
	 _setSdaHigh(this);
	 delayMicroseconds(_delay_us);
	 _setSclHigh(this);
	 */
	softI2C_stop(si2cp);
}

void softI2C_stop(const softI2CDriver *si2cp) {
	// Force SCL low
	// Why SCL low here?
//	softI2C_setSclLow(si2cp);
//	gptPolledDelay(&GPTD4,softI2C_delay_us);

	// Force SDA low
	softI2C_setSdaLow(si2cp);
	gptPolledDelay(&GPTD4, softI2C_delay_us);

	// Release SCL
	softI2C_setSclHigh(si2cp);
	gptPolledDelay(&GPTD4, softI2C_delay_us);

	// Release SDA
	softI2C_setSdaHigh(si2cp);
	gptPolledDelay(&GPTD4, softI2C_delay_us);
}

result_t softI2C_llStart(const softI2CDriver *si2cp, uint8_t rawAddr) {
	// Force SDA low
	softI2C_setSdaLow(si2cp);
	gptPolledDelay(&GPTD4, softI2C_delay_us);

	// Force SCL low
	softI2C_setSclLow(si2cp);
	gptPolledDelay(&GPTD4, softI2C_delay_us);

	return softI2C_write(si2cp, rawAddr);
}

result_t softI2C_llRepeatedStart(const softI2CDriver *si2cp, uint8_t rawAddr) {
	// Force SCL low
	softI2C_setSclLow(si2cp);
	gptPolledDelay(&GPTD4, softI2C_delay_us);

	// Release SDA
	softI2C_setSdaHigh(si2cp);
	gptPolledDelay(&GPTD4, softI2C_delay_us);

	// Release SCL
	softI2C_setSclHigh(si2cp);
	gptPolledDelay(&GPTD4, softI2C_delay_us);

	// Force SDA low
	softI2C_setSdaLow(si2cp);
	gptPolledDelay(&GPTD4, softI2C_delay_us);

	return softI2C_write(si2cp, rawAddr);
}

result_t softI2C_llStartWait(const softI2CDriver *si2cp, uint8_t rawAddr) {
	systime_t timeout_start = chVTGetSystemTime();

	while (chVTTimeElapsedSinceX(timeout_start) <= softI2C_timeout) {
		// Force SDA low
		softI2C_setSdaLow(si2cp);
		gptPolledDelay(&GPTD4, softI2C_delay_us);

		switch (softI2C_write(si2cp, rawAddr)) {
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

result_t softI2C_write(const softI2CDriver *si2cp, uint8_t data) {
	systime_t timeout_start = chVTGetSystemTime();

	for (uint8_t i = 8; i; --i) {
		// Force SCL low
		softI2C_setSclLow(si2cp);

		if (data & 0x80) { // MSB
		// Release SDA
			softI2C_setSdaHigh(si2cp);
		} else {
		// Force SDA low
			softI2C_setSdaLow(si2cp);
		}
		gptPolledDelay(&GPTD4, softI2C_delay_us);

		// Release SCL
		softI2C_setSclHigh(si2cp);
		gptPolledDelay(&GPTD4, softI2C_delay_us);

		data <<= 1;

		if (chVTTimeElapsedSinceX(timeout_start) > softI2C_timeout) {
			softI2C_stop(si2cp); // Reset bus
			return timedOut;
		}
	}

	// Get ACK
	// Force SCL low
	softI2C_setSclLow(si2cp);

	// Release SDA
	softI2C_setSdaHigh(si2cp);
	gptPolledDelay(&GPTD4, softI2C_delay_us);

	// Release SCL
	softI2C_setSclHigh(si2cp);

	// Wait for SCL to be set high (in case wait states are inserted)
	while (softI2C_readScl(si2cp) == 0) {
		if (chVTTimeElapsedSinceX(timeout_start) > softI2C_timeout) {
			softI2C_stop(si2cp); // Reset bus
			return timedOut;
		}
	}

	result_t res = (softI2C_readSda(si2cp) == 0 ? ack : nack);

	gptPolledDelay(&GPTD4, softI2C_delay_us);

	// Keep SCL low between bytes
	softI2C_setSclLow(si2cp);

	return res;
}

result_t softI2C_read(const softI2CDriver *si2cp, uint8_t *data, _Bool sendAck) {
	*data = 0;
	systime_t timeout_start = chVTGetSystemTime();

	for (uint8_t i = 8; i; --i) {
		*data <<= 1;

		// Force SCL low
		softI2C_setSclLow(si2cp);

		// Release SDA (from previous ACK)
		softI2C_setSdaHigh(si2cp);
		gptPolledDelay(&GPTD4, softI2C_delay_us);

		// Release SCL
		softI2C_setSclHigh(si2cp);
		gptPolledDelay(&GPTD4, softI2C_delay_us);

		// Read clock stretch
		while (softI2C_readScl(si2cp) == 0)
			if (chVTTimeElapsedSinceX(timeout_start) > softI2C_timeout) {
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

	gptPolledDelay(&GPTD4, softI2C_delay_us);

	// Release SCL
	softI2C_setSclHigh(si2cp);

	// Wait for SCL to return high
	while (softI2C_readScl(si2cp) == 0) {
		if (chVTTimeElapsedSinceX(timeout_start) > softI2C_timeout) {
			softI2C_stop(si2cp); // Reset bus
			return timedOut;
		}
	}

	gptPolledDelay(&GPTD4, softI2C_delay_us);

	// Keep SCL low between bytes
	softI2C_setSclLow(si2cp);

	return ack;
}



