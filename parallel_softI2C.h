/*
 * parallel_softI2C.h
 *
 *  Created on: 31 Dec 2017
 *      Author: m2
 */

#ifndef PARALLEL_SOFTI2C_H_
#define PARALLEL_SOFTI2C_H_

#include <stdint.h>
#include "ch.h"
#include "hal.h"

#include "softI2C.h"

typedef uint16_t i2caddr_t;
typedef uint32_t systime_t;

typedef struct parallel_softI2CDriver {
	softI2CDriver SI2CD[2];
	uint8_t driver_num;
} parallel_softI2CDriver;

void parallel_softi2cMasterTransmitTimeout(softI2CDriver *i2cp,
		i2caddr_t addr, // 7-bit address
		const uint8_t *txbuf, size_t txbytes, uint8_t *rxbuf, size_t rxbytes,
		systime_t timeout);

void parallel_softi2cMasterReceiveTimeout(softI2CDriver *i2cp, i2caddr_t addr, // 7-bit address
		uint8_t *rxbuf, size_t rxbytes, systime_t timeout);

void parallel_softI2C_setSdaLow(const softI2CDriver *si2cp);
void parallel_softI2C_setSdaHigh(const softI2CDriver *si2cp);
void parallel_softI2C_setSclLow(const softI2CDriver *si2cp);
void parallel_softI2C_setSclHigh(const softI2CDriver *si2cp);
_Bool parallel_softI2C_readSda(const softI2CDriver *si2cp);
_Bool parallel_softI2C_readScl(const softI2CDriver *si2cp);

 // Functions which take raw addresses (ie address passed must
 // already indicate read/write mode)
result_t parallel_softI2C_llStart(const softI2CDriver *si2cp, uint8_t rawAddr);
result_t parallel_softI2C_llRepeatedStart(const softI2CDriver *si2cp,
		uint8_t rawAddr);
result_t parallel_softI2C_llStartWait(const softI2CDriver *si2cp,
		uint8_t rawAddr);
void parallel_softI2C_begin(const softI2CDriver *si2cp);
void parallel_softI2C_stop(const softI2CDriver *si2cp);

result_t parallel_softI2C_write(const softI2CDriver *si2cp, uint8_t data);
result_t parallel_softI2C_read(const softI2CDriver *si2cp, uint8_t *data,
		_Bool sendAck);
 
#endif /* PARALLEL_SOFTI2C_H_ */
