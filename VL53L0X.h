/*
 * VL53L0X.h
 *
 *  Created on: 29 Dec 2017
 *      Author: kevin-pololu
 */

#ifndef VL53L0X_H_
#define VL53L0X_H_

#include "ch.h"
#include "hal.h"
#include "softI2C.h"

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define ADDRESS_DEFAULT 0b0101001

#define SYSRANGE_START                              0x00

#define SYSTEM_THRESH_HIGH                          0x0C
#define SYSTEM_THRESH_LOW                           0x0E

#define SYSTEM_SEQUENCE_CONFIG                      0x01
#define SYSTEM_RANGE_CONFIG                         0x09
#define SYSTEM_INTERMEASUREMENT_PERIOD              0x04

#define SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A

#define GPIO_HV_MUX_ACTIVE_HIGH                     0x84

#define SYSTEM_INTERRUPT_CLEAR                      0x0B

#define RESULT_INTERRUPT_STATUS                     0x13
#define RESULT_RANGE_STATUS                         0x14

#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       0xBC
#define RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        0xC0
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       0xD0
#define RESULT_CORE_RANGING_TOTAL_EVENTS_REF        0xD4
#define RESULT_PEAK_SIGNAL_RATE_REF                 0xB6

#define ALGO_PART_TO_PART_RANGE_OFFSET_MM           0x28

#define I2C_SLAVE_DEVICE_ADDRESS                    0x8A

#define MSRC_CONFIG_CONTROL                         0x60

#define PRE_RANGE_CONFIG_MIN_SNR                    0x27
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW            0x56
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH           0x57
#define PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          0x64

#define FINAL_RANGE_CONFIG_MIN_SNR                  0x67
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW          0x47
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         0x48
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44

#define PRE_RANGE_CONFIG_SIGMA_THRESH_HI            0x61
#define PRE_RANGE_CONFIG_SIGMA_THRESH_LO            0x62

#define PRE_RANGE_CONFIG_VCSEL_PERIOD               0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x51
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          0x52

#define SYSTEM_HISTOGRAM_BIN                        0x81
#define HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       0x33
#define HISTOGRAM_CONFIG_READOUT_CTRL               0x55

#define FINAL_RANGE_CONFIG_VCSEL_PERIOD             0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        0x71
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        0x72
#define CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       0x20

#define MSRC_CONFIG_TIMEOUT_MACROP                  0x46

#define SOFT_RESET_GO2_SOFT_RESET_N                 0xBF
#define IDENTIFICATION_MODEL_ID                     0xC0
#define IDENTIFICATION_REVISION_ID                  0xC2

#define OSC_CALIBRATE_VAL                           0xF8

#define GLOBAL_CONFIG_VCSEL_WIDTH                   0x32
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_1            0xB1
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_2            0xB2
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_3            0xB3
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_4            0xB4
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_5            0xB5

#define GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F
#define POWER_MANAGEMENT_GO1_POWER_FORCE            0x80

#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89

#define ALGO_PHASECAL_LIM                           0x30
#define ALGO_PHASECAL_CONFIG_TIMEOUT                0x30

typedef struct VL53L0X_board {
	uint8_t address;
	I2CDriver* I2CD;
	ioportid_t xshut_port;
	uint8_t xshut_pad;
} VL53L0X_board;

/* Defines VL53L0X boards */
#define VL53L0X_COUNT 16
extern VL53L0X_board VB[VL53L0X_COUNT];

#define VL53L0X_I2C_TIMEOUT TIME_INFINITE//MS2ST(10)

typedef enum VL53L0X_profile {
	VL53L0X_HighAccuracy, VL53L0X_HighSpeed //, VL53L0X_LongRange
} VL53L0X_profile;

typedef enum VL53L0X_vcselPeriodType {
	VcselPeriodPreRange, VcselPeriodFinalRange
} VL53L0X_vcselPeriodType;

void VL53L0X_setAddress(VL53L0X_board vb);
// inline uint8_t getAddress(void) { return address; }

_Bool VL53L0X_init(VL53L0X_board vb, _Bool io_2v8);
void VL53L0X_setProfile(VL53L0X_board vb, VL53L0X_profile profile);

void VL53L0X_writeReg(VL53L0X_board vb, uint8_t reg, uint8_t value);
void VL53L0X_writeReg16Bit(VL53L0X_board vb, uint8_t reg, uint16_t value);
void VL53L0X_writeReg32Bit(VL53L0X_board vb, uint8_t reg, uint32_t value);
uint8_t VL53L0X_readReg(VL53L0X_board vb, uint8_t reg);
uint16_t VL53L0X_readReg16Bit(VL53L0X_board vb, uint8_t reg);
// uint32_t readReg32Bit(uint8_t reg);

void VL53L0X_writeMulti(VL53L0X_board vb, uint8_t reg, uint8_t const * src,
		uint8_t count);
void VL53L0X_readMulti(VL53L0X_board vb, uint8_t reg, uint8_t * dst,
		uint8_t count);

bool VL53L0X_setSignalRateLimit(VL53L0X_board vb, float limit_Mcps);
// float getSignalRateLimit(void);

_Bool VL53L0X_setMeasurementTimingBudget(VL53L0X_board vb, uint32_t budget_us);
uint32_t VL53L0X_getMeasurementTimingBudget(VL53L0X_board vb);

// bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);
uint8_t VL53L0X_getVcselPulsePeriod(VL53L0X_board vb,
		VL53L0X_vcselPeriodType type);

void VL53L0X_startContinuous(VL53L0X_board vb);
//void VL53L0X_startContinuous(uint32_t period_ms);
// void stopContinuous(void);
uint16_t VL53L0X_readRangeContinuousMillimeters(VL53L0X_board vb);
void VL53L0X_readRangeContinuousMillimeters_loop(VL53L0X_board vb[],
		uint32_t count, uint32_t timeout);
uint16_t VL53L0X_readRangeSingleMillimeters(VL53L0X_board vb);

void VL53L0X_setTimeout(uint16_t timeout);
//uint16_t VL53L0X_getTimeout(void);
//_Bool VL53L0X_timeoutOccurred(void);
/*

 // TCC: Target CentreCheck
 // MSRC: Minimum Signal Rate Check
 // DSS: Dynamic Spad Selection
 */
typedef struct VL53L0X_SequenceStepEnables {
	_Bool tcc, msrc, dss, pre_range, final_range;
} VL53L0X_SequenceStepEnables;

typedef struct VL53L0X_SequenceStepTimeouts {
 uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

 uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
 uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
} VL53L0X_SequenceStepTimeouts;

_Bool VL53L0X_getSpadInfo(VL53L0X_board vb, uint8_t * count,
		_Bool * type_is_aperture);

void VL53L0X_getSequenceStepEnables(VL53L0X_board vb,
		VL53L0X_SequenceStepEnables * enables);
void VL53L0X_getSequenceStepTimeouts(VL53L0X_board vb,
		VL53L0X_SequenceStepEnables const * enables,
		VL53L0X_SequenceStepTimeouts * timeouts);

_Bool VL53L0X_performSingleRefCalibration(VL53L0X_board vb,
		uint8_t vhv_init_byte);

uint16_t VL53L0X_decodeTimeout(uint16_t reg_val);
uint16_t VL53L0X_encodeTimeout(uint16_t timeout_mclks);
uint32_t VL53L0X_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks,
		uint8_t vcsel_period_pclks);
uint32_t VL53L0X_timeoutMicrosecondsToMclks(uint32_t timeout_period_us,
		uint8_t vcsel_period_pclks);

#endif /* VL53L0X_H_ */
