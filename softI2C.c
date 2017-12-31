/*
 * softI2C.c
 *
 *  Created on: 31 Dec 2017
 *      Author: m2
 */

#include <softI2C.h>

//void I2Cstart(I2CDriver *i2cp, const I2CConfig *config) {
void softI2Cstart(softI2CDriver *si2cp) {
	palSetPadMode(si2cp->sdaPort, si2cp->sdaPad, PAL_MODE_OUTPUT_OPENDRAIN);
	palSetPadMode(si2cp->sclPort, si2cp->sckPad, PAL_MODE_OUTPUT_OPENDRAIN);

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
uint8_t softI2C_readSda(const softI2CDriver *si2cp) {
	osalSysLock();
	uint8_t value = palReadPad(si2cp->sdaPort, si2cp->sdaPad);
	osalSysUnlock();
	return value;
 }

 // Read SCL (to detect clock-stretching)
uint8_t softI2C_readScl(const softI2CDriver *si2cp) {
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
void softI2C_begin(void) {
	/*
	 // Release SDA and SCL
	 _setSdaHigh(this);
	 delayMicroseconds(_delay_us);
	 _setSclHigh(this);
	 */
	softI2C_stop();
}

void softI2C_stop(const softI2CDriver *si2cp) {
	// Force SCL low
	softI2C_setSclLow(si2cp);
	chThdSleepMicroseconds(_delay_us);

	// Force SDA low
	softI2C_setSdaLow(si2cp);
	chThdSleepMicroseconds(_delay_us);

	// Release SCL
	softI2C_setSclHigh(si2cp);
	chThdSleepMicroseconds(_delay_us);

	// Release SDA
	softI2C_setSdaHigh(si2cp);
	chThdSleepMicroseconds(_delay_us);
}

result_t softI2C_llStart(const softI2CDriver *si2cp, uint8_t rawAddr) {
	// Force SDA low
	softI2C_setSdaLow(si2cp);
	chThdSleepMicroseconds(_delay_us);

	// Force SCL low
	softI2C_setSclLow(si2cp);
	chThdSleepMicroseconds(_delay_us);

	return softI2C_write(rawAddr);
}

result_t softI2C_llRepeatedStart(const softI2CDriver *si2cp, uint8_t rawAddr) {
	// Force SCL low
	softI2C_setSclLow(si2cp);
	chThdSleepMicroseconds(_delay_us);

	// Release SDA
	softI2C_setSdaHigh(si2cp);
	chThdSleepMicroseconds(_delay_us);

	// Release SCL
	softI2C_setSclHigh(si2cp);
	chThdSleepMicroseconds(_delay_us);

	// Force SDA low
	softI2C_setSdaLow(si2cp);
	chThdSleepMicroseconds(_delay_us);

	return softI2C_write(rawAddr);
}
/*
SoftWire::result_t SoftWire::llStartWait(uint8_t rawAddr) const
{
AsyncDelay timeout(_timeout_ms, AsyncDelay::MILLIS);

while (!timeout.isExpired()) {
	// Force SDA low
	_setSdaLow(this);
	delayMicroseconds(_delay_us);

	switch (write(rawAddr)) {
		case ack:
		return ack;
		case nack:
		stop();
		default:
		// timeout, and anything else we don't know about
		stop();
		return timedOut;
	}
}
return timedOut;
}
 */
result_t softI2C_write(const softI2CDriver *si2cp, uint8_t data) {
	systime_t timeout_start = chVTGetSystemTime();
	//AsyncDelay timeout(_timeout_ms, AsyncDelay::MILLIS);
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
		delayMicroseconds(_delay_us);

		// Release SCL
		softI2C_setSclHigh(si2cp);
		chThdSleepMicroseconds(_delay_us);

		data <<= 1;

		if (chVTTimeElapsedSinceX(timeout_start) > MS2ST(_timeout_ms)) {
			softI2C_stop(); // Reset bus
			return timedOut;
		}
	}

	// Get ACK
	// Force SCL low
	softI2C_setSclLow(si2cp);

	// Release SDA
	softI2C_setSdaHigh(si2cp);
	chThdSleepMicroseconds(_delay_us);

	// Release SCL
	softI2C_setSclHigh(si2cp);

	// Wait for SCL to be set high (in case wait states are inserted)
	while (softI2C_readScl(si2cp) == 0) {
		if (chVTTimeElapsedSinceX(timeout_start) > MS2ST(_timeout_ms)) {
			stop(); // Reset bus
			return timedOut;
		}
	}

	result_t res = (_readSda(this) == LOW ? ack : nack);

	chThdSleepMicroseconds(_delay_us);

	// Keep SCL low between bytes
	softI2C_setSclLow(si2cp);

	return res;
}
/*
SoftWire::result_t SoftWire::read(uint8_t &data, bool sendAck) const
{
data = 0;
AsyncDelay timeout(_timeout_ms, AsyncDelay::MILLIS);

for (uint8_t i = 8; i; --i) {
	data <<= 1;

	// Force SCL low
	_setSclLow(this);

	// Release SDA (from previous ACK)
	_setSdaHigh(this);
	delayMicroseconds(_delay_us);

	// Release SCL
	_setSclHigh(this);
	delayMicroseconds(_delay_us);

	// Read clock stretch
	while (_readScl(this) == LOW)
	if (timeout.isExpired()) {
		stop(); // Reset bus
		return timedOut;
	}

	if (_readSda(this))
	data |= 1;
}

// Put ACK/NACK

// Force SCL low
_setSclLow(this);
if (sendAck) {
	// Force SDA low
	_setSdaLow(this);
}
else {
	// Release SDA
	_setSdaHigh(this);
}

delayMicroseconds(_delay_us);

// Release SCL
_setSclHigh(this);
delayMicroseconds(_delay_us);

// Wait for SCL to return high
while (_readScl(this) == LOW)
if (timeout.isExpired()) {
	stop(); // Reset bus
	return timedOut;
}

delayMicroseconds(_delay_us);

// Keep SCL low between bytes
_setSclLow(this);

return ack;
}
*/


