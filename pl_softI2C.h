/*
 * pl_softI2C.h
 *
 *  Created on: 31 Dec 2017
 *      Author: m2
 */

#ifndef PL_SOFTI2C_H_
#define PL_SOFTI2C_H_

#include <stdint.h>
#include "ch.h"
#include "hal.h"

#include "softI2C.h"

typedef uint16_t i2caddr_t;
typedef uint32_t systime_t;

typedef struct pl_softI2CDriver {
	softI2CDriver SI2CD[16];
	uint8_t driver_num;
} pl_softI2CDriver;

extern pl_softI2CDriver PI2CD1;

void pl_softi2cMasterTransmitTimeout(softI2CDriver *i2cp,
		i2caddr_t addr, // 7-bit address
		const uint8_t *txbuf, size_t txbytes, uint8_t *rxbuf, size_t rxbytes,
		systime_t timeout);

void pl_softi2cMasterReceiveTimeout(softI2CDriver *i2cp, i2caddr_t addr, // 7-bit address
		uint8_t *rxbuf, size_t rxbytes, systime_t timeout);

_Bool pl_softi2c_none_alive(pl_softI2CDriver *psi2cp);
void pl_softI2C_set_alive(const pl_softI2CDriver *psi2cp);

void pl_softI2C_setSdaLow(const pl_softI2CDriver *psi2cp);
void pl_softI2C_setSdaHigh(const pl_softI2CDriver *psi2cp);
void pl_softI2C_setSclLow(const pl_softI2CDriver *psi2cp);
void pl_softI2C_setSclHigh(const pl_softI2CDriver *psi2cp);
uint16_t pl_softI2C_readSda(const pl_softI2CDriver *psi2cp);
uint16_t pl_softI2C_readScl(const pl_softI2CDriver *psi2cp);
//void pl_softI2C_readScl(const pl_softI2CDriver *psi2cp, _Bool value[]);

 // Functions which take raw addresses (ie address passed must
 // already indicate read/write mode)
void pl_softI2C_llRepeatedStart(const pl_softI2CDriver *psi2cp,
		uint8_t rawAddr);
void pl_softI2C_llStartWait(const pl_softI2CDriver *psi2cp, uint8_t rawAddr);

void pl_softI2C_stop(const pl_softI2CDriver *psi2cp);

void pl_softI2C_write(const pl_softI2CDriver *psi2cp, uint8_t data);
void pl_softI2C_read(const pl_softI2CDriver *psi2cp, uint8_t data[],
_Bool sendAck);

#endif /* PL_SOFTI2C_H_ */
