/*
 * VL53L0X.c
 *
 *  Created on: 29 Dec 2017
 *  Author: kevin-pololu
 *
 *  Most of the functionality of this library is based on the VL53L0X API
 *  provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
 *  or paraphrased from the API source code, API user manual (UM2039), and the
 *  VL53L0X datasheet.
 */

#include "VL53L0X.h"
#include "chprintf.h"

VL53L0X_board VB[VL53L0X_COUNT] = { { 0b0101010, &I2CD1, GPIOB, 5U }, {
		0b0101011, &I2CD1, GPIOB, 4U }, { 0b0101100, &I2CD1, GPIOB, 10U }, {
		0b0101101, &I2CD1, GPIOA, 8U }, { 0b0101110, &I2CD1, GPIOA, 9U }, {
		0b0101111, &I2CD1, GPIOC, 7U }, { 0b0110000, &I2CD1, GPIOB, 6U }, {
		0b0110001, &I2CD1, GPIOA, 7U }, { 0b0110010, &I2CD1, GPIOC, 0U }, {
		0b0110011, &I2CD1, GPIOC, 1U }, { 0b0110100, &I2CD1, GPIOB, 0U }, {
		0b0110101, &I2CD1, GPIOA, 4U }, { 0b0110110, &I2CD1, GPIOA, 1U }, {
		0b0110111, &I2CD1, GPIOB, 15U }, { 0b0111000, &I2CD1, GPIOB, 14U }, {
		0b0111001, &I2CD1, GPIOB, 13U } };

msg_t VL53L0X_last_status; // status of last I2C transmission

uint8_t VL53L0X_address = ADDRESS_DEFAULT;
uint32_t VL53L0X_io_timeout = 0;
//_Bool VL53L0X_did_timeout;
systime_t VL53L0X_timeout_start;

uint8_t VL53L0X_stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
uint32_t VL53L0X_measurement_timing_budget_us;

// Defines /////////////////////////////////////////////////////////////////////

// Record the current time to check an upcoming timeout against
#define VL53L0X_startTimeout() (VL53L0X_timeout_start = chVTGetSystemTime())

// Check if timeout is enabled (set to nonzero value) and has expired
#define VL53L0X_checkTimeoutExpired(timeout) (timeout > 0 && chVTTimeElapsedSinceX(VL53L0X_timeout_start) > MS2ST(timeout))

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define VL53L0X_decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define VL53L0X_encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define VL53L0X_calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

void VL53L0X_setAddress(VL53L0X_board vb) {
	uint8_t txbuf[2] = { I2C_SLAVE_DEVICE_ADDRESS, vb.address & 0x7F };
//	VL53L0X_last_status = i2cMasterTransmitTimeout(vb.I2CD, ADDRESS_DEFAULT,
//			txbuf, 2, NULL, 0, TIME_INFINITE);
	VL53L0X_last_status = i2cMasterTransmitTimeout(vb.I2CD, ADDRESS_DEFAULT,
			txbuf, 2, NULL, 0, VL53L0X_I2C_TIMEOUT);
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
_Bool VL53L0X_init(VL53L0X_board vb, _Bool io_2v8) {
	// VL53L0X_DataInit() begin

	// sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
	if (io_2v8) {
		VL53L0X_writeReg(vb, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
				VL53L0X_readReg(vb, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
	}

	// "Set I2C standard mode"
	VL53L0X_writeReg(vb, 0x88, 0x00);

	VL53L0X_writeReg(vb, 0x80, 0x01);
	VL53L0X_writeReg(vb, 0xFF, 0x01);
	VL53L0X_writeReg(vb, 0x00, 0x00);
	VL53L0X_stop_variable = VL53L0X_readReg(vb, 0x91);
	VL53L0X_writeReg(vb, 0x00, 0x01);
	VL53L0X_writeReg(vb, 0xFF, 0x00);
	VL53L0X_writeReg(vb, 0x80, 0x00);

	// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	VL53L0X_writeReg(vb, MSRC_CONFIG_CONTROL,
			VL53L0X_readReg(vb, MSRC_CONFIG_CONTROL) | 0x12);

	// set final range signal rate limit to 0.25 MCPS (million counts per second)
	VL53L0X_setSignalRateLimit(vb, 0.25);

	VL53L0X_writeReg(vb, SYSTEM_SEQUENCE_CONFIG, 0xFF);

	// VL53L0X_DataInit() end

	// VL53L0X_StaticInit() begin

	uint8_t spad_count;
	_Bool spad_type_is_aperture;
	if (!VL53L0X_getSpadInfo(vb, &spad_count, &spad_type_is_aperture)) {
		return false;
	}

	// The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
	// the API, but the same data seems to be more easily readable from
	// GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	uint8_t ref_spad_map[6];
	VL53L0X_readMulti(vb, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	// -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

	VL53L0X_writeReg(vb, 0xFF, 0x01);
	VL53L0X_writeReg(vb, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	VL53L0X_writeReg(vb, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	VL53L0X_writeReg(vb, 0xFF, 0x00);
	VL53L0X_writeReg(vb, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

	uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
	uint8_t spads_enabled = 0;

	for (uint8_t i = 0; i < 48; i++) {
		if (i < first_spad_to_enable || spads_enabled == spad_count) {
			// This bit is lower than the first one that should be enabled, or
			// (reference_spad_count) bits have already been enabled, so zero this bit
			ref_spad_map[i / 8] &= ~(1 << (i % 8));
		} else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
			spads_enabled++;
		}
	}

	VL53L0X_writeMulti(vb, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	// -- VL53L0X_set_reference_spads() end

	// -- VL53L0X_load_tuning_settings() begin
	// DefaultTuningSettings from vl53l0x_tuning.h

	VL53L0X_writeReg(vb, 0xFF, 0x01);
	VL53L0X_writeReg(vb, 0x00, 0x00);

	VL53L0X_writeReg(vb, 0xFF, 0x00);
	VL53L0X_writeReg(vb, 0x09, 0x00);
	VL53L0X_writeReg(vb, 0x10, 0x00);
	VL53L0X_writeReg(vb, 0x11, 0x00);

	VL53L0X_writeReg(vb, 0x24, 0x01);
	VL53L0X_writeReg(vb, 0x25, 0xFF);
	VL53L0X_writeReg(vb, 0x75, 0x00);

	VL53L0X_writeReg(vb, 0xFF, 0x01);
	VL53L0X_writeReg(vb, 0x4E, 0x2C);
	VL53L0X_writeReg(vb, 0x48, 0x00);
	VL53L0X_writeReg(vb, 0x30, 0x20);

	VL53L0X_writeReg(vb, 0xFF, 0x00);
	VL53L0X_writeReg(vb, 0x30, 0x09);
	VL53L0X_writeReg(vb, 0x54, 0x00);
	VL53L0X_writeReg(vb, 0x31, 0x04);
	VL53L0X_writeReg(vb, 0x32, 0x03);
	VL53L0X_writeReg(vb, 0x40, 0x83);
	VL53L0X_writeReg(vb, 0x46, 0x25);
	VL53L0X_writeReg(vb, 0x60, 0x00);
	VL53L0X_writeReg(vb, 0x27, 0x00);
	VL53L0X_writeReg(vb, 0x50, 0x06);
	VL53L0X_writeReg(vb, 0x51, 0x00);
	VL53L0X_writeReg(vb, 0x52, 0x96);
	VL53L0X_writeReg(vb, 0x56, 0x08);
	VL53L0X_writeReg(vb, 0x57, 0x30);
	VL53L0X_writeReg(vb, 0x61, 0x00);
	VL53L0X_writeReg(vb, 0x62, 0x00);
	VL53L0X_writeReg(vb, 0x64, 0x00);
	VL53L0X_writeReg(vb, 0x65, 0x00);
	VL53L0X_writeReg(vb, 0x66, 0xA0);

	VL53L0X_writeReg(vb, 0xFF, 0x01);
	VL53L0X_writeReg(vb, 0x22, 0x32);
	VL53L0X_writeReg(vb, 0x47, 0x14);
	VL53L0X_writeReg(vb, 0x49, 0xFF);
	VL53L0X_writeReg(vb, 0x4A, 0x00);

	VL53L0X_writeReg(vb, 0xFF, 0x00);
	VL53L0X_writeReg(vb, 0x7A, 0x0A);
	VL53L0X_writeReg(vb, 0x7B, 0x00);
	VL53L0X_writeReg(vb, 0x78, 0x21);

	VL53L0X_writeReg(vb, 0xFF, 0x01);
	VL53L0X_writeReg(vb, 0x23, 0x34);
	VL53L0X_writeReg(vb, 0x42, 0x00);
	VL53L0X_writeReg(vb, 0x44, 0xFF);
	VL53L0X_writeReg(vb, 0x45, 0x26);
	VL53L0X_writeReg(vb, 0x46, 0x05);
	VL53L0X_writeReg(vb, 0x40, 0x40);
	VL53L0X_writeReg(vb, 0x0E, 0x06);
	VL53L0X_writeReg(vb, 0x20, 0x1A);
	VL53L0X_writeReg(vb, 0x43, 0x40);

	VL53L0X_writeReg(vb, 0xFF, 0x00);
	VL53L0X_writeReg(vb, 0x34, 0x03);
	VL53L0X_writeReg(vb, 0x35, 0x44);

	VL53L0X_writeReg(vb, 0xFF, 0x01);
	VL53L0X_writeReg(vb, 0x31, 0x04);
	VL53L0X_writeReg(vb, 0x4B, 0x09);
	VL53L0X_writeReg(vb, 0x4C, 0x05);
	VL53L0X_writeReg(vb, 0x4D, 0x04);

	VL53L0X_writeReg(vb, 0xFF, 0x00);
	VL53L0X_writeReg(vb, 0x44, 0x00);
	VL53L0X_writeReg(vb, 0x45, 0x20);
	VL53L0X_writeReg(vb, 0x47, 0x08);
	VL53L0X_writeReg(vb, 0x48, 0x28);
	VL53L0X_writeReg(vb, 0x67, 0x00);
	VL53L0X_writeReg(vb, 0x70, 0x04);
	VL53L0X_writeReg(vb, 0x71, 0x01);
	VL53L0X_writeReg(vb, 0x72, 0xFE);
	VL53L0X_writeReg(vb, 0x76, 0x00);
	VL53L0X_writeReg(vb, 0x77, 0x00);

	VL53L0X_writeReg(vb, 0xFF, 0x01);
	VL53L0X_writeReg(vb, 0x0D, 0x01);

	VL53L0X_writeReg(vb, 0xFF, 0x00);
	VL53L0X_writeReg(vb, 0x80, 0x01);
	VL53L0X_writeReg(vb, 0x01, 0xF8);

	VL53L0X_writeReg(vb, 0xFF, 0x01);
	VL53L0X_writeReg(vb, 0x8E, 0x01);
	VL53L0X_writeReg(vb, 0x00, 0x01);
	VL53L0X_writeReg(vb, 0xFF, 0x00);
	VL53L0X_writeReg(vb, 0x80, 0x00);

	// -- VL53L0X_load_tuning_settings() end

	// "Set interrupt config to new sample ready"
	// -- VL53L0X_SetGpioConfig() begin

	VL53L0X_writeReg(vb, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
	VL53L0X_writeReg(vb, GPIO_HV_MUX_ACTIVE_HIGH,
			VL53L0X_readReg(vb, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
	VL53L0X_writeReg(vb, SYSTEM_INTERRUPT_CLEAR, 0x01);

	// -- VL53L0X_SetGpioConfig() end

	VL53L0X_measurement_timing_budget_us = VL53L0X_getMeasurementTimingBudget(
			vb);

	// "Disable MSRC and TCC by default"
	// MSRC = Minimum Signal Rate Check
	// TCC = Target CentreCheck
	// -- VL53L0X_SetSequenceStepEnable() begin

	VL53L0X_writeReg(vb, SYSTEM_SEQUENCE_CONFIG, 0xE8);

	// -- VL53L0X_SetSequenceStepEnable() end

	// "Recalculate timing budget"
	VL53L0X_setMeasurementTimingBudget(vb,
			VL53L0X_measurement_timing_budget_us);

	// VL53L0X_StaticInit() end

	// VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

	// -- VL53L0X_perform_vhv_calibration() begin

	VL53L0X_writeReg(vb, SYSTEM_SEQUENCE_CONFIG, 0x01);
	if (!VL53L0X_performSingleRefCalibration(vb, 0x40)) {
		return false;
	}

	// -- VL53L0X_perform_vhv_calibration() end

	// -- VL53L0X_perform_phase_calibration() begin

	VL53L0X_writeReg(vb, SYSTEM_SEQUENCE_CONFIG, 0x02);
	if (!VL53L0X_performSingleRefCalibration(vb, 0x00)) {
		return false;
	}

	// -- VL53L0X_perform_phase_calibration() end

	// "restore the previous Sequence Config"
	VL53L0X_writeReg(vb, SYSTEM_SEQUENCE_CONFIG, 0xE8);

	// VL53L0X_PerformRefCalibration() end

	return true;
}

void VL53L0X_setProfile(VL53L0X_board vb, VL53L0X_profile profile) {
	switch (profile) {

	case VL53L0X_LongRange:
		// lower the return signal rate limit (default is 0.25 MCPS)
		VL53L0X_setSignalRateLimit(vb, 0.1);
		// increase laser pulse periods (defaults are 14 and 10 PCLKs)
		VL53L0X_setVcselPulsePeriod(vb, VcselPeriodPreRange, 18);
		VL53L0X_setVcselPulsePeriod(vb, VcselPeriodFinalRange, 14);
		break;

	case VL53L0X_HighSpeed:
		VL53L0X_setMeasurementTimingBudget(vb, 20000);
		break;
	case VL53L0X_HighAccuracy:
		VL53L0X_setMeasurementTimingBudget(vb, 200000);
		break;
	}
}

// Write an 8-bit register
void VL53L0X_writeReg(VL53L0X_board vb, uint8_t reg, uint8_t value) {
	uint8_t txbuf[2];
	txbuf[0] = reg;
	txbuf[1] = value;
	VL53L0X_last_status = i2cMasterTransmitTimeout(vb.I2CD, vb.address, txbuf,
			2, NULL, 0, VL53L0X_I2C_TIMEOUT);
}

// Write a 16-bit register
void VL53L0X_writeReg16Bit(VL53L0X_board vb, uint8_t reg, uint16_t value) {
	uint8_t txbuf[3];
	txbuf[0] = reg;
	txbuf[1] = (value >> 8) & 0xFF;
	txbuf[2] = value & 0xFF;
	VL53L0X_last_status = i2cMasterTransmitTimeout(vb.I2CD, vb.address, txbuf,
			3, NULL, 0, VL53L0X_I2C_TIMEOUT);
}

// Write a 32-bit register
void VL53L0X_writeReg32Bit(VL53L0X_board vb, uint8_t reg, uint32_t value) {
	uint8_t txbuf[5];
	txbuf[0] = reg;
	txbuf[1] = (value >> 24) & 0xFF;
	txbuf[2] = (value >> 16) & 0xFF;
	txbuf[3] = (value >> 8) & 0xFF;
	txbuf[4] = value & 0xFF;
	VL53L0X_last_status = i2cMasterTransmitTimeout(vb.I2CD, vb.address, txbuf,
			3, NULL, 0, VL53L0X_I2C_TIMEOUT);
}

// Read an 8-bit register
uint8_t VL53L0X_readReg(VL53L0X_board vb, uint8_t reg) {
	uint8_t rxbuf = 0;
	VL53L0X_last_status = i2cMasterTransmitTimeout(vb.I2CD, vb.address, &reg, 1,
			&rxbuf, 1, VL53L0X_I2C_TIMEOUT);
	return rxbuf;
}

// Read a 16-bit register
uint16_t VL53L0X_readReg16Bit(VL53L0X_board vb, uint8_t reg) {
	uint8_t rxbuf[2] = { 0, 0 };
	VL53L0X_last_status = i2cMasterTransmitTimeout(vb.I2CD, vb.address, &reg, 1,
			rxbuf, 2, VL53L0X_I2C_TIMEOUT);

	uint16_t value;
	value = (uint16_t) rxbuf[0] << 8; // value high byte
	value |= rxbuf[1];      // value low byte
	return value;
}
/*
 // Read a 32-bit register
 uint32_t VL53L0X::readReg32Bit(uint8_t reg)
 {
 uint32_t value;

 Wire.beginTransmission(address);
 Wire.write(reg);
 last_status = Wire.endTransmission();

 Wire.requestFrom(address, (uint8_t)4);
 value  = (uint32_t)Wire.read() << 24; // value highest byte
 value |= (uint32_t)Wire.read() << 16;
 value |= (uint16_t)Wire.read() <<  8;
 value |=           Wire.read();       // value lowest byte

 return value;
 }
 */
// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void VL53L0X_writeMulti(VL53L0X_board vb, uint8_t reg, uint8_t const * src,
		uint8_t count) {

	uint8_t txbuf[count + 1];
	txbuf[0] = reg;
	for (int i = 0; i < count; i++) {
		txbuf[i + 1] = src[i];
	}

	VL53L0X_last_status = i2cMasterTransmitTimeout(vb.I2CD, vb.address, txbuf,
			count + 1, NULL, 0, VL53L0X_I2C_TIMEOUT);

}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void VL53L0X_readMulti(VL53L0X_board vb, uint8_t reg, uint8_t * dst,
		uint8_t count) {
	VL53L0X_last_status = i2cMasterTransmitTimeout(vb.I2CD, vb.address, &reg, 1,
			dst, count, VL53L0X_I2C_TIMEOUT);
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool VL53L0X_setSignalRateLimit(VL53L0X_board vb, float limit_Mcps) {
	if (limit_Mcps < 0 || limit_Mcps > 511.99) {
		return false;
	}

	// Q9.7 fixed point format (9 integer bits, 7 fractional bits)
	VL53L0X_writeReg16Bit(vb, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
			limit_Mcps * (1 << 7));
	return true;
}
/*
 // Get the return signal rate limit check value in MCPS
 float VL53L0X::getSignalRateLimit(void)
 {
 return (float)readReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
 }
 */
// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
_Bool VL53L0X_setMeasurementTimingBudget(VL53L0X_board vb, uint32_t budget_us) {
	VL53L0X_SequenceStepEnables enables;
	VL53L0X_SequenceStepTimeouts timeouts;

	uint16_t const StartOverhead = 1320; // note that this is different than the value in get_
	uint16_t const EndOverhead = 960;
	uint16_t const MsrcOverhead = 660;
	uint16_t const TccOverhead = 590;
	uint16_t const DssOverhead = 690;
	uint16_t const PreRangeOverhead = 660;
	uint16_t const FinalRangeOverhead = 550;

	uint32_t const MinTimingBudget = 20000;

	if (budget_us < MinTimingBudget) {
		return false;
	}

	uint32_t used_budget_us = StartOverhead + EndOverhead;

	VL53L0X_getSequenceStepEnables(vb, &enables);
	VL53L0X_getSequenceStepTimeouts(vb, &enables, &timeouts);

	if (enables.tcc) {
		used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}

	if (enables.dss) {
		used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	} else if (enables.msrc) {
		used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}

	if (enables.pre_range) {
		used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	}

	if (enables.final_range) {
		used_budget_us += FinalRangeOverhead;

		// "Note that the final range timeout is determined by the timing
		// budget and the sum of all other timeouts within the sequence.
		// If there is no room for the final range timeout, then an error
		// will be set. Otherwise the remaining time will be applied to
		// the final range."

		if (used_budget_us > budget_us) {
			// "Requested timeout too big."
			return false;
		}

		uint32_t final_range_timeout_us = budget_us - used_budget_us;

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

		// "For the final range timeout, the pre-range timeout
		//  must be added. To do this both final and pre-range
		//  timeouts must be expressed in macro periods MClks
		//  because they have different vcsel periods."

		uint16_t final_range_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(
				final_range_timeout_us,
				timeouts.final_range_vcsel_period_pclks);

		if (enables.pre_range) {
			final_range_timeout_mclks += timeouts.pre_range_mclks;
		}

		VL53L0X_writeReg16Bit(vb, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
				VL53L0X_encodeTimeout(final_range_timeout_mclks));

		// set_sequence_step_timeout() end

		VL53L0X_measurement_timing_budget_us = budget_us; // store for internal reuse
	}
	return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t VL53L0X_getMeasurementTimingBudget(VL53L0X_board vb) {
	VL53L0X_SequenceStepEnables enables;
	VL53L0X_SequenceStepTimeouts timeouts;

	uint16_t const StartOverhead = 1910; // note that this is different than the value in set_
	uint16_t const EndOverhead = 960;
	uint16_t const MsrcOverhead = 660;
	uint16_t const TccOverhead = 590;
	uint16_t const DssOverhead = 690;
	uint16_t const PreRangeOverhead = 660;
	uint16_t const FinalRangeOverhead = 550;

	// "Start and end overhead times always present"
	uint32_t budget_us = StartOverhead + EndOverhead;

	VL53L0X_getSequenceStepEnables(vb, &enables);
	VL53L0X_getSequenceStepTimeouts(vb, &enables, &timeouts);

	if (enables.tcc) {
		budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}

	if (enables.dss) {
		budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	} else if (enables.msrc) {
		budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}

	if (enables.pre_range) {
		budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	}

	if (enables.final_range) {
		budget_us += (timeouts.final_range_us + FinalRangeOverhead);
	}

	VL53L0X_measurement_timing_budget_us = budget_us; // store for internal reuse
	return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
_Bool VL53L0X_setVcselPulsePeriod(VL53L0X_board vb,
		VL53L0X_vcselPeriodType type, uint8_t period_pclks) {
	uint8_t vcsel_period_reg = VL53L0X_encodeVcselPeriod(period_pclks);

	VL53L0X_SequenceStepEnables enables;
	VL53L0X_SequenceStepTimeouts timeouts;

	VL53L0X_getSequenceStepEnables(vb, &enables);
	VL53L0X_getSequenceStepTimeouts(vb, &enables, &timeouts);

// "Apply specific settings for the requested clock period"
// "Re-calculate and apply timeouts, in macro periods"

// "When the VCSEL period for the pre or final range is changed,
// the corresponding timeout must be read from the device using
// the current VCSEL period, then the new VCSEL period can be
// applied. The timeout then must be written back to the device
// using the new VCSEL period.
//
// For the MSRC timeout, the same applies - this timeout being
// dependant on the pre-range vcsel period."

	if (type == VcselPeriodPreRange) {
// "Set phase check limits"
		switch (period_pclks) {
		case 12:
			VL53L0X_writeReg(vb, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
			break;

		case 14:
			VL53L0X_writeReg(vb, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
			break;

		case 16:
			VL53L0X_writeReg(vb, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
			break;

		case 18:
			VL53L0X_writeReg(vb, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
			break;

		default:
// invalid period
			return false;
		}
		VL53L0X_writeReg(vb, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

// apply new VCSEL period
		VL53L0X_writeReg(vb, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

// update timeouts

// set_sequence_step_timeout() begin
// (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

		uint16_t new_pre_range_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(
				timeouts.pre_range_us, period_pclks);

		VL53L0X_writeReg16Bit(vb, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
				VL53L0X_encodeTimeout(new_pre_range_timeout_mclks));

// set_sequence_step_timeout() end

// set_sequence_step_timeout() begin
// (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

		uint16_t new_msrc_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(
				timeouts.msrc_dss_tcc_us, period_pclks);

		VL53L0X_writeReg(vb, MSRC_CONFIG_TIMEOUT_MACROP,
				(new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

// set_sequence_step_timeout() end
	} else if (type == VcselPeriodFinalRange) {
		switch (period_pclks) {
		case 8:
			VL53L0X_writeReg(vb, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
			VL53L0X_writeReg(vb, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
			VL53L0X_writeReg(vb, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
			VL53L0X_writeReg(vb, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
			VL53L0X_writeReg(vb, 0xFF, 0x01);
			VL53L0X_writeReg(vb, ALGO_PHASECAL_LIM, 0x30);
			VL53L0X_writeReg(vb, 0xFF, 0x00);
			break;

		case 10:
			VL53L0X_writeReg(vb, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
			VL53L0X_writeReg(vb, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
			VL53L0X_writeReg(vb, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
			VL53L0X_writeReg(vb, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
			VL53L0X_writeReg(vb, 0xFF, 0x01);
			VL53L0X_writeReg(vb, ALGO_PHASECAL_LIM, 0x20);
			VL53L0X_writeReg(vb, 0xFF, 0x00);
			break;

		case 12:
			VL53L0X_writeReg(vb, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
			VL53L0X_writeReg(vb, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
			VL53L0X_writeReg(vb, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
			VL53L0X_writeReg(vb, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
			VL53L0X_writeReg(vb, 0xFF, 0x01);
			VL53L0X_writeReg(vb, ALGO_PHASECAL_LIM, 0x20);
			VL53L0X_writeReg(vb, 0xFF, 0x00);
			break;

		case 14:
			VL53L0X_writeReg(vb, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
			VL53L0X_writeReg(vb, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
			VL53L0X_writeReg(vb, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
			VL53L0X_writeReg(vb, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
			VL53L0X_writeReg(vb, 0xFF, 0x01);
			VL53L0X_writeReg(vb, ALGO_PHASECAL_LIM, 0x20);
			VL53L0X_writeReg(vb, 0xFF, 0x00);
			break;

		default:
// invalid period
			return false;
		}

// apply new VCSEL period
		VL53L0X_writeReg(vb, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

// update timeouts

// set_sequence_step_timeout() begin
// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

// "For the final range timeout, the pre-range timeout
//  must be added. To do this both final and pre-range
//  timeouts must be expressed in macro periods MClks
//  because they have different vcsel periods."

		uint16_t new_final_range_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(
				timeouts.final_range_us, period_pclks);

		if (enables.pre_range) {
			new_final_range_timeout_mclks += timeouts.pre_range_mclks;
		}

		VL53L0X_writeReg16Bit(vb, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
				VL53L0X_encodeTimeout(new_final_range_timeout_mclks));

// set_sequence_step_timeout end
	} else {
// invalid type
		return false;
	}

// "Finally, the timing budget must be re-applied"

	VL53L0X_setMeasurementTimingBudget(vb, VL53L0X_measurement_timing_budget_us);

// "Perform the phase calibration. This is needed after changing on vcsel period."
// VL53L0X_perform_phase_calibration() begin

	uint8_t sequence_config = VL53L0X_readReg(vb, SYSTEM_SEQUENCE_CONFIG);
	VL53L0X_writeReg(vb, SYSTEM_SEQUENCE_CONFIG, 0x02);
	VL53L0X_performSingleRefCalibration(vb, 0x0);
	VL53L0X_writeReg(vb, SYSTEM_SEQUENCE_CONFIG, sequence_config);

// VL53L0X_perform_phase_calibration() end

	return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t VL53L0X_getVcselPulsePeriod(VL53L0X_board vb,
		VL53L0X_vcselPeriodType type) {
	if (type == VcselPeriodPreRange) {
		return VL53L0X_decodeVcselPeriod(
				VL53L0X_readReg(vb,PRE_RANGE_CONFIG_VCSEL_PERIOD));
	} else if (type == VcselPeriodFinalRange) {
		return VL53L0X_decodeVcselPeriod(
				VL53L0X_readReg(vb,FINAL_RANGE_CONFIG_VCSEL_PERIOD));
	} else {
		return 255;
	}
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()

//void VL53L0X_startContinuous(uint32_t period_ms) {
void VL53L0X_startContinuous(VL53L0X_board vb) {
	uint32_t period_ms = 0;
	VL53L0X_writeReg(vb, 0x80, 0x01);
	VL53L0X_writeReg(vb, 0xFF, 0x01);
	VL53L0X_writeReg(vb, 0x00, 0x00);
	VL53L0X_writeReg(vb, 0x91, VL53L0X_stop_variable);
	VL53L0X_writeReg(vb, 0x00, 0x01);
	VL53L0X_writeReg(vb, 0xFF, 0x00);
	VL53L0X_writeReg(vb, 0x80, 0x00);

	if (period_ms != 0) {
		// continuous timed mode

		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

		uint16_t osc_calibrate_val = VL53L0X_readReg16Bit(vb,
		OSC_CALIBRATE_VAL);

		if (osc_calibrate_val != 0) {
			period_ms *= osc_calibrate_val;
		}

		VL53L0X_writeReg32Bit(vb, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

		VL53L0X_writeReg(vb, SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
	} else {
		// continuous back-to-back mode
		VL53L0X_writeReg(vb, SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
	}
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
/*void VL53L0X::stopContinuous(void)
 {
 writeReg(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

 writeReg(0xFF, 0x01);
 writeReg(0x00, 0x00);
 writeReg(0x91, 0x00);
 writeReg(0x00, 0x01);
 writeReg(0xFF, 0x00);
 }*/

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t VL53L0X_readRangeContinuousMillimeters(VL53L0X_board vb) {
	VL53L0X_startTimeout();
	while ((VL53L0X_readReg(vb, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
		if (VL53L0X_checkTimeoutExpired(VL53L0X_io_timeout)) {
//			VL53L0X_did_timeout = true;
			return 65535;
		}
	}

	// assumptions: Linearity Corrective Gain is 1000 (default);
	// fractional ranging is not enabled
	uint16_t range = VL53L0X_readReg16Bit(vb, RESULT_RANGE_STATUS + 10);

	VL53L0X_writeReg(vb, SYSTEM_INTERRUPT_CLEAR, 0x01);

	return range;
}

void VL53L0X_readRangeContinuousMillimeters_loop(VL53L0X_board vb[],
		uint32_t count, uint32_t timeout) {
	VL53L0X_startTimeout();
	uint32_t success[VL53L0X_COUNT] = { };
	uint32_t range[VL53L0X_COUNT] = { };
	uint32_t fail[VL53L0X_COUNT] = { };
	uint16_t min[VL53L0X_COUNT] = { 8179, 8179, 8179, 8179, 8179, 8179, 8179,
			8179, 8179, 8179, 8179, 8179, 8179, 8179, 8179, 8179 };
	uint16_t max[VL53L0X_COUNT] = { };

	while (!VL53L0X_checkTimeoutExpired(timeout)) {
		for (uint32_t i = 0; i < count; i++) {
			if ((VL53L0X_readReg(vb[i], RESULT_INTERRUPT_STATUS) & 0x07) != 0) {
				// assumptions: Linearity Corrective Gain is 1000 (default);
				// fractional ranging is not enabled
				uint16_t current_range = VL53L0X_readReg16Bit(vb[i],
						RESULT_RANGE_STATUS + 10);
				if (current_range < 8190) {
					range[i] += current_range;
					success[i]++;

					if (current_range < min[i]) {
						min[i] = current_range;
					}
					if (current_range > max[i]) {
						max[i] = current_range;
					}
				} else {
					fail[i]++;
				}

				VL53L0X_writeReg(vb[i], SYSTEM_INTERRUPT_CLEAR, 0x01);

//				BaseSequentialStream* chp = (BaseSequentialStream*) &SD2;
//				chprintf(chp, "%2d %d\r\n", i, range);
			}
		}
	}
	BaseSequentialStream* chp = (BaseSequentialStream*) &SD2;
	for (uint32_t i = 0; i < count; i++) {
		if (success[i] > 0) {
			chprintf(chp, "%2d: %5d %2d %2d %2d %2d ", i, range[i] / success[i],
					min[i], max[i], success[i], fail[i]);
		} else {
			chprintf(chp, "%2d:       %2d %2d %2d %2d ", i, min[i], max[i],
					success[i], fail[i]);
		}
	}
	chprintf(chp, "\r\n\n");
}

 // Performs a single-shot range measurement and returns the reading in
 // millimeters
 // based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t VL53L0X_readRangeSingleMillimeters(VL53L0X_board vb) {
	VL53L0X_writeReg(vb, 0x80, 0x01);
	VL53L0X_writeReg(vb, 0xFF, 0x01);
	VL53L0X_writeReg(vb, 0x00, 0x00);
	VL53L0X_writeReg(vb, 0x91, VL53L0X_stop_variable);
	VL53L0X_writeReg(vb, 0x00, 0x01);
	VL53L0X_writeReg(vb, 0xFF, 0x00);
	VL53L0X_writeReg(vb, 0x80, 0x00);

	VL53L0X_writeReg(vb, SYSRANGE_START, 0x01);

	// "Wait until start bit has been cleared"
	VL53L0X_startTimeout();
	while (VL53L0X_readReg(vb, SYSRANGE_START) & 0x01) {
		if (VL53L0X_checkTimeoutExpired(VL53L0X_io_timeout)) {
			return 65535;
		}
	}

	return VL53L0X_readRangeContinuousMillimeters(vb);
}

void VL53L0X_setTimeout(uint16_t timeout) {
	VL53L0X_io_timeout = timeout;
}
/*
 uint16_t VL53L0X_getTimeout(void) {
 return io_timeout;
 }
 */
// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
//_Bool VL53L0X_timeoutOccurred() {
//	_Bool tmp = VL53L0X_did_timeout;
//	VL53L0X_did_timeout = false;
//	return tmp;
//}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
_Bool VL53L0X_getSpadInfo(VL53L0X_board vb, uint8_t * count,
		_Bool * type_is_aperture) {
	uint8_t tmp;

	VL53L0X_writeReg(vb, 0x80, 0x01);
	VL53L0X_writeReg(vb, 0xFF, 0x01);
	VL53L0X_writeReg(vb, 0x00, 0x00);

	VL53L0X_writeReg(vb, 0xFF, 0x06);
	VL53L0X_writeReg(vb, 0x83, VL53L0X_readReg(vb, 0x83) | 0x04);
	VL53L0X_writeReg(vb, 0xFF, 0x07);
	VL53L0X_writeReg(vb, 0x81, 0x01);

	VL53L0X_writeReg(vb, 0x80, 0x01);

	VL53L0X_writeReg(vb, 0x94, 0x6b);
	VL53L0X_writeReg(vb, 0x83, 0x00);
	VL53L0X_startTimeout();
	while (VL53L0X_readReg(vb, 0x83) == 0x00) {
		if (VL53L0X_checkTimeoutExpired(VL53L0X_io_timeout)) {
			return false;
		}
	}
	VL53L0X_writeReg(vb, 0x83, 0x01);
	tmp = VL53L0X_readReg(vb, 0x92);

	*count = tmp & 0x7f;
	*type_is_aperture = (tmp >> 7) & 0x01;

	VL53L0X_writeReg(vb, 0x81, 0x00);
	VL53L0X_writeReg(vb, 0xFF, 0x06);
	VL53L0X_writeReg(vb, 0x83, VL53L0X_readReg(vb, 0x83) & ~0x04);
	VL53L0X_writeReg(vb, 0xFF, 0x01);
	VL53L0X_writeReg(vb, 0x00, 0x01);

	VL53L0X_writeReg(vb, 0xFF, 0x00);
	VL53L0X_writeReg(vb, 0x80, 0x00);

	return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void VL53L0X_getSequenceStepEnables(VL53L0X_board vb,
		VL53L0X_SequenceStepEnables * enables) {
	uint8_t sequence_config = VL53L0X_readReg(vb, SYSTEM_SEQUENCE_CONFIG);

	enables->tcc = (sequence_config >> 4) & 0x1;
	enables->dss = (sequence_config >> 3) & 0x1;
	enables->msrc = (sequence_config >> 2) & 0x1;
	enables->pre_range = (sequence_config >> 6) & 0x1;
	enables->final_range = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void VL53L0X_getSequenceStepTimeouts(VL53L0X_board vb,
		VL53L0X_SequenceStepEnables const * enables,
		VL53L0X_SequenceStepTimeouts * timeouts) {
	timeouts->pre_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(vb,
			VcselPeriodPreRange);

	timeouts->msrc_dss_tcc_mclks = VL53L0X_readReg(vb,
	MSRC_CONFIG_TIMEOUT_MACROP) + 1;
	timeouts->msrc_dss_tcc_us = VL53L0X_timeoutMclksToMicroseconds(
			timeouts->msrc_dss_tcc_mclks,
			timeouts->pre_range_vcsel_period_pclks);

	timeouts->pre_range_mclks = VL53L0X_decodeTimeout(
			VL53L0X_readReg16Bit(vb, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
	timeouts->pre_range_us = VL53L0X_timeoutMclksToMicroseconds(
			timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

	timeouts->final_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(vb,
			VcselPeriodFinalRange);

	timeouts->final_range_mclks = VL53L0X_decodeTimeout(
			VL53L0X_readReg16Bit(vb, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

	if (enables->pre_range) {
		timeouts->final_range_mclks -= timeouts->pre_range_mclks;
	}

	timeouts->final_range_us = VL53L0X_timeoutMclksToMicroseconds(
			timeouts->final_range_mclks,
			timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t VL53L0X_decodeTimeout(uint16_t reg_val) {
	// format: "(LSByte * 2^MSByte) + 1"
	return (uint16_t) ((reg_val & 0x00FF)
			<< (uint16_t) ((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t VL53L0X_encodeTimeout(uint16_t timeout_mclks) {
	// format: "(LSByte * 2^MSByte) + 1"

	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;

	if (timeout_mclks > 0) {
		ls_byte = timeout_mclks - 1;

		while ((ls_byte & 0xFFFFFF00) > 0) {
			ls_byte >>= 1;
			ms_byte++;
		}

		return (ms_byte << 8) | (ls_byte & 0xFF);
	} else {
		return 0;
	}
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t VL53L0X_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks,
		uint8_t vcsel_period_pclks) {
	uint32_t macro_period_ns = VL53L0X_calcMacroPeriod(vcsel_period_pclks);
	return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2))
			/ 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t VL53L0X_timeoutMicrosecondsToMclks(uint32_t timeout_period_us,
		uint8_t vcsel_period_pclks) {
	uint32_t macro_period_ns = VL53L0X_calcMacroPeriod(vcsel_period_pclks);
	return (((timeout_period_us * 1000) + (macro_period_ns / 2))
			/ macro_period_ns);
}

// based on VL53L0X_perform_single_ref_calibration()
_Bool VL53L0X_performSingleRefCalibration(VL53L0X_board vb,
		uint8_t vhv_init_byte) {
	VL53L0X_writeReg(vb, SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

	VL53L0X_startTimeout();
	while ((VL53L0X_readReg(vb,
	RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
		if (VL53L0X_checkTimeoutExpired(VL53L0X_io_timeout)) {
			return false;
		}
	}

	VL53L0X_writeReg(vb, SYSTEM_INTERRUPT_CLEAR, 0x01);
	VL53L0X_writeReg(vb, SYSRANGE_START, 0x00);

	return true;
}

