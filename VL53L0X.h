/*
 * VL53L0X.h
 *
 *  Created on: 29 Dec 2017
 *      Author: kevin-pololu
 */

#ifndef VL53L0X_H_
#define VL53L0X_H_

#include "ch.h"

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

//enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };

msg_t VL53L0X_last_status; // status of last I2C transmission

/*
 void setAddress(uint8_t new_addr);
 inline uint8_t getAddress(void) { return address; }

 bool init(bool io_2v8 = true);

 void writeReg(uint8_t reg, uint8_t value);
 void writeReg16Bit(uint8_t reg, uint16_t value);
 void writeReg32Bit(uint8_t reg, uint32_t value);
 uint8_t readReg(uint8_t reg);
 uint16_t readReg16Bit(uint8_t reg);
 uint32_t readReg32Bit(uint8_t reg);

 void writeMulti(uint8_t reg, uint8_t const * src, uint8_t count);
 void readMulti(uint8_t reg, uint8_t * dst, uint8_t count);

 bool setSignalRateLimit(float limit_Mcps);
 float getSignalRateLimit(void);

 bool setMeasurementTimingBudget(uint32_t budget_us);
 uint32_t getMeasurementTimingBudget(void);

 bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);
 uint8_t getVcselPulsePeriod(vcselPeriodType type);

 void startContinuous(uint32_t period_ms = 0);
 void stopContinuous(void);
 uint16_t readRangeContinuousMillimeters(void);
 uint16_t readRangeSingleMillimeters(void);
 */
inline void VL53L0X_setTimeout(uint16_t timeout) {
	VL53L0X_io_timeout = timeout;
}
/*
 inline uint16_t getTimeout(void) { return io_timeout; }
 bool timeoutOccurred(void);


 // TCC: Target CentreCheck
 // MSRC: Minimum Signal Rate Check
 // DSS: Dynamic Spad Selection
 */
 struct SequenceStepEnables
 {
	_Bool tcc, msrc, dss, pre_range, final_range;
 };

 struct SequenceStepTimeouts
 {
 uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

 uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
 uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
 };
 */
/*
uint8_t VL53L0X_address = ADDRESS_DEFAULT;
uint32_t VL53L0X_io_timeout = 0;
//bool did_timeout;
systime_t VL53L0X_timeout_start;

uint8_t VL53L0X_stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
uint32_t VL53L0X_measurement_timing_budget_us;
/*
bool getSpadInfo(uint8_t * count, bool * type_is_aperture);

void getSequenceStepEnables(SequenceStepEnables * enables);
void getSequenceStepTimeouts(SequenceStepEnables const * enables,
		SequenceStepTimeouts * timeouts);

bool performSingleRefCalibration(uint8_t vhv_init_byte);

static uint16_t decodeTimeout(uint16_t value);
static uint16_t encodeTimeout(uint16_t timeout_mclks);
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks,
		uint8_t vcsel_period_pclks);
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us,
		uint8_t vcsel_period_pclks);
};
 */
#endif /* VL53L0X_H_ */
