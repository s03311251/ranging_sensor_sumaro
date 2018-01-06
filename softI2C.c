/*
 * softI2C.c
 *
 *  Created on: 31 Dec 2017
 *      Author: m2
 */

#include "softI2C.h"

uint8_t softI2C_delay_us = defaultDelay_us;
uint32_t softI2C_timeout = defaultTimeout;

//void I2Cstart(I2CDriver *i2cp, const I2CConfig *config) {
/*void softI2Cstart(softI2CDriver *si2cp) {
	palSetPadMode(si2cp->sdaPort, si2cp->sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
	palSetPadMode(si2cp->sclPort, si2cp->sckPad, PAL_MODE_OUTPUT_OPENDRAIN);

}
 */

void softi2cMasterTransmitTimeout(softI2CDriver *si2cp,
		i2caddr_t addr, // 7-bit address
		const uint8_t *txbuf, size_t txbytes, uint8_t *rxbuf, size_t rxbytes,
		uint32_t timeout) {
	softI2C_timeout = timeout;

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
		uint8_t *rxbuf, size_t rxbytes, uint32_t timeout) {
	softI2C_timeout = timeout;
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
//	chThdSleepMicroseconds(softI2C_delay_us);

	// Force SDA low
	softI2C_setSdaLow(si2cp);
	chThdSleepMicroseconds(softI2C_delay_us);

	// Release SCL
	softI2C_setSclHigh(si2cp);
	chThdSleepMicroseconds(softI2C_delay_us);

	// Release SDA
	softI2C_setSdaHigh(si2cp);
	chThdSleepMicroseconds(softI2C_delay_us);
}

result_t softI2C_llStart(const softI2CDriver *si2cp, uint8_t rawAddr) {
	// Force SDA low
	softI2C_setSdaLow(si2cp);
	chThdSleepMicroseconds(softI2C_delay_us);

	// Force SCL low
	softI2C_setSclLow(si2cp);
	chThdSleepMicroseconds(softI2C_delay_us);

	return softI2C_write(si2cp, rawAddr);
}

result_t softI2C_llRepeatedStart(const softI2CDriver *si2cp, uint8_t rawAddr) {
	// Force SCL low
	softI2C_setSclLow(si2cp);
	chThdSleepMicroseconds(softI2C_delay_us);

	// Release SDA
	softI2C_setSdaHigh(si2cp);
	chThdSleepMicroseconds(softI2C_delay_us);

	// Release SCL
	softI2C_setSclHigh(si2cp);
	chThdSleepMicroseconds(softI2C_delay_us);

	// Force SDA low
	softI2C_setSdaLow(si2cp);
	chThdSleepMicroseconds(softI2C_delay_us);

	return softI2C_write(si2cp, rawAddr);
}

result_t softI2C_llStartWait(const softI2CDriver *si2cp, uint8_t rawAddr) {
	//systime_t timeout_start = chVTGetSystemTime();
	uint32_t timeout_start = chVTGetSystemTime();

	while (chVTTimeElapsedSinceX(timeout_start) <= softI2C_timeout) {
		// Force SDA low
		softI2C_setSdaLow(si2cp);
		chThdSleepMicroseconds(softI2C_delay_us);

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
	//systime_t timeout_start = chVTGetSystemTime();
	uint32_t timeout_start = chVTGetSystemTime();
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
		chThdSleepMicroseconds(softI2C_delay_us);

		// Release SCL
		softI2C_setSclHigh(si2cp);
		chThdSleepMicroseconds(softI2C_delay_us);

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
	chThdSleepMicroseconds(softI2C_delay_us);

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

	chThdSleepMicroseconds(softI2C_delay_us);

	// Keep SCL low between bytes
	softI2C_setSclLow(si2cp);

	return res;
}

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
		chThdSleepMicroseconds(softI2C_delay_us);

		// Release SCL
		softI2C_setSclHigh(si2cp);
		chThdSleepMicroseconds(softI2C_delay_us);

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

	chThdSleepMicroseconds(softI2C_delay_us);
	
	// Release SCL
	softI2C_setSclHigh(si2cp);

	// Wait for SCL to return high
	while (softI2C_readScl(si2cp) == 0) {
		if (chVTTimeElapsedSinceX(timeout_start) > softI2C_timeout) {
			softI2C_stop(si2cp); // Reset bus
			return timedOut;
		}
	}

	chThdSleepMicroseconds(softI2C_delay_us);

	// Keep SCL low between bytes
	softI2C_setSclLow(si2cp);

	return ack;
}



