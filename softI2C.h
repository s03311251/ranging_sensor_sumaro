/*
 * softI2C.h
 *
 *  Created on: 31 Dec 2017
 *      Author: m2
 */

#ifndef SOFTI2C_H_
#define SOFTI2C_H_

#include <stdint.h>
#include "ch.h"
#include "hal.h"

typedef uint16_t i2caddr_t;
typedef uint32_t systime_t;

#define defaultDelay_us 1
#define defaultTimeout MS2ST(1) // 1ms

typedef struct softI2CDriver {
	//const softI2CConfig *config;
	ioportid_t sdaPort;
	uint8_t sdaPad;
	ioportid_t sclPort;
	uint8_t sclPad;
} softI2CDriver;

typedef enum result_t {
	ack = 0, nack = 1, timedOut = 2,
} result_t;

extern softI2CDriver SI2CD1;
extern softI2CDriver SI2CD2;

msg_t softi2cMasterTransmitTimeout(softI2CDriver *i2cp,
		i2caddr_t addr, // 7-bit address
		const uint8_t *txbuf, size_t txbytes, uint8_t *rxbuf, size_t rxbytes,
		systime_t timeout);

msg_t softi2cMasterReceiveTimeout(softI2CDriver *i2cp, i2caddr_t addr, // 7-bit address
		uint8_t *rxbuf, size_t rxbytes, systime_t timeout);

void softI2C_setSdaLow(const softI2CDriver *si2cp);
void softI2C_setSdaHigh(const softI2CDriver *si2cp);
void softI2C_setSclLow(const softI2CDriver *si2cp);
void softI2C_setSclHigh(const softI2CDriver *si2cp);
_Bool softI2C_readSda(const softI2CDriver *si2cp);
_Bool softI2C_readScl(const softI2CDriver *si2cp);
/*
 // SMBus uses CRC-8 for its PEC
 static uint8_t crc8_update(uint8_t crc, uint8_t data);

 SoftWire(uint8_t sda, uint8_t scl);
 inline uint8_t getSda(void) const;
 inline uint8_t getScl(void) const;
 inline uint8_t getDelay_us(void) const;
 inline uint16_t getTimeout_ms(void) const;
 inline uint8_t getInputMode(void) const;

 // begin() must be called after any changes are made to SDA and/or
 // SCL pins.
 inline void setSda(uint8_t sda);
 inline void setScl(uint8_t scl);
 inline void enablePullups(bool enablePullups = true);

 inline void setDelay_us(uint8_t delay_us);
 inline void setTimeout_ms(uint16_t timeout_ms);

 // begin() must be called before use, and after any changes are made
 // to the SDA and/or SCL pins.
 void begin(void) const;

 */
 // Functions which take raw addresses (ie address passed must
 // already indicate read/write mode)
result_t softI2C_llStart(const softI2CDriver *si2cp, uint8_t rawAddr);
result_t softI2C_llRepeatedStart(const softI2CDriver *si2cp, uint8_t rawAddr);
result_t softI2C_llStartWait(const softI2CDriver *si2cp, uint8_t rawAddr);
void softI2C_begin(const softI2CDriver *si2cp);
void softI2C_stop(const softI2CDriver *si2cp);
/*
 inline result_t startRead(uint8_t addr) const;
 inline result_t startWrite(uint8_t addr) const;
 inline result_t repeatedStartRead(uint8_t addr) const;
 inline result_t repeatedStartWrite(uint8_t addr) const;
 inline result_t startReadWait(uint8_t addr) const;
 inline result_t startWriteWait(uint8_t addr) const;

 inline result_t start(uint8_t addr, mode_t rwMode) const;
 inline result_t repeatedStart(uint8_t addr, mode_t rwMode) const;
 inline result_t startWait(uint8_t addr, mode_t rwMode) const;
 */
result_t softI2C_write(const softI2CDriver *si2cp, uint8_t data);
result_t softI2C_read(const softI2CDriver *si2cp, uint8_t *data, _Bool sendAck);
/*inline result_t readThenAck(uint8_t &data) const;
 inline result_t readThenNack(uint8_t &data) const;

 inline void setSdaLow(void) const;
 inline void setSdaHigh(void) const;
 inline void setSclLow(void) const;
 inline void setSclHigh(void) const;



 private:
 uint8_t _sda;
 uint8_t _scl;
 uint8_t _inputMode;*/
/*
 public:
 void (*_setSdaLow)(const SoftWire *p);
 void (*_setSdaHigh)(const SoftWire *p);
 void (*_setSclLow)(const SoftWire *p);
 void (*_setSclHigh)(const SoftWire *p);
 uint8_t (*_readSda)(const SoftWire *p);
 uint8_t (*_readScl)(const SoftWire *p);
 };


 uint8_t SoftWire::getSda(void) const
 {
 return _sda;
 }


 uint8_t SoftWire::getScl(void) const
 {
 return _scl;
 }


 uint8_t SoftWire::getDelay_us(void) const
 {
 return _delay_us;
 }

 uint16_t SoftWire::getTimeout_ms(void) const
 {
 return _timeout_ms;
 }

 uint8_t SoftWire::getInputMode(void) const
 {
 return _inputMode;
 }

 void SoftWire::setSda(uint8_t sda)
 {
 _sda = sda;
 }


 void SoftWire::setScl(uint8_t scl)
 {
 _scl = scl;
 }


 void SoftWire::enablePullups(bool enable)
 {
 _inputMode = (enable ? INPUT_PULLUP : INPUT);
 }


 void SoftWire::setDelay_us(uint8_t delay_us)
 {
 _delay_us = delay_us;
 }


 void SoftWire::setTimeout_ms(uint16_t timeout_ms) {
 _timeout_ms = timeout_ms;
 }


 SoftWire::result_t SoftWire::startRead(uint8_t addr) const
 {
 return llStart((addr << 1) + readMode);
 }


 SoftWire::result_t SoftWire::startWrite(uint8_t addr) const
 {
 return llStart((addr << 1) + writeMode);
 }


 SoftWire::result_t SoftWire::repeatedStartRead(uint8_t addr) const
 {
 return llRepeatedStart((addr << 1) + readMode);
 }


 SoftWire::result_t SoftWire::repeatedStartWrite(uint8_t addr) const
 {
 return llRepeatedStart((addr << 1) + writeMode);
 }


 SoftWire::result_t SoftWire::startReadWait(uint8_t addr) const
 {
 return llStartWait((addr << 1) + readMode);
 }


 SoftWire::result_t SoftWire::startWriteWait(uint8_t addr) const
 {
 return llStartWait((addr << 1) + writeMode);
 }


 SoftWire::result_t SoftWire::start(uint8_t addr, mode_t rwMode) const
 {
 return llStart((addr << 1) + rwMode);
 }


 SoftWire::result_t SoftWire::repeatedStart(uint8_t addr, mode_t rwMode) const
 {
 return llRepeatedStart((addr << 1) + rwMode);
 }


 SoftWire::result_t SoftWire::startWait(uint8_t addr, mode_t rwMode) const
 {
 return llStartWait((addr << 1) + rwMode);
 }


 SoftWire::result_t SoftWire::readThenAck(uint8_t &data) const
 {
 return read(data, true);
 }


 SoftWire::result_t SoftWire::readThenNack(uint8_t &data) const
 {
 return read(data, false);
 }


 void SoftWire::setSdaLow(void) const
 {
 _setSdaLow(this);
 }


 void SoftWire::setSdaHigh(void) const
 {
 _setSdaHigh(this);
 }


 void SoftWire::setSclLow(void) const
 {
 _setSclLow(this);
 }


 void SoftWire::setSclHigh(void) const
 {
 _setSclHigh(this);
 }
 */

#endif /* SOFTI2C_H_ */
