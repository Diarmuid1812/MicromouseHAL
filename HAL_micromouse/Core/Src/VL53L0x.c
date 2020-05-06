//VL53L0x.c
#include "VL53L0x.h"
#include "i2c.h"


//Zapisywanie do rejestrów

void ToF_writeReg(struct ToF_struct *ToF, uint8_t reg, uint8_t value)
{
	uint8_t data[2];

    data[0] = reg;
    data[1] = value;

	HAL_I2C_Master_Transmit(&hi2c1, (ToF->bus_address)<<1, data, 2, 100);
}

void ToF_writeReg16Bit(struct ToF_struct *ToF, uint8_t reg, uint16_t value)
{
    uint8_t data[3];

    data[0] = reg;
    data[1] = (uint8_t)(value >> 8);    // MSB
    data[2] = (uint8_t)(value     );    // LSB

    HAL_I2C_Master_Transmit(&hi2c1, (ToF->bus_address)<<1, data, 3, 100);
}

void ToF_writeReg32Bit(struct ToF_struct *ToF, uint8_t reg, uint32_t value)
{
    uint8_t data[5];

    data[0] = reg;
	data[1]=(uint8_t)(value >> 24); //MSB
	data[2]=(uint8_t)(value >> 16);
	data[3]=(uint8_t)(value >> 8 );
	data[4]=(uint8_t)(value      ); //LSB

    HAL_I2C_Master_Transmit(&hi2c1, (ToF->bus_address)<<1, data, 5, 100);
}

void ToF_writeMulti(struct ToF_struct *ToF, uint8_t reg, uint8_t *src, uint8_t count)
{
	uint8_t data[count+1];

	data[0] = reg;
	data[1] = 211;
	data[2] = 0;
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;

/*
	data[0] = reg;
	for(int i = 0; i < count; i++)
	{
		data[i+1] == src[count-i];
	}
*/

    HAL_I2C_Master_Transmit(&hi2c1, (ToF->bus_address)<<1, data, count+1, 100);
}


//czytanie z rejestrów
uint8_t ToF_readReg(struct ToF_struct *ToF, uint8_t reg)
{
	uint8_t value = 0;

    HAL_I2C_Master_Transmit(&hi2c1, (ToF->bus_address)<<1, &reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, (ToF->bus_address)<<1,  &value, 1, 100);

    return value;
}

uint16_t ToF_readReg16Bit(struct ToF_struct *ToF, uint8_t reg)
{
	uint8_t value_tab[2];

    HAL_I2C_Master_Transmit(&hi2c1, (ToF->bus_address)<<1, &reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, (ToF->bus_address)<<1,  value_tab, 2, 100);

	uint16_t value = (uint16_t) value_tab[0] << 8;
	value |= value_tab[1];

	return value;
}

void ToF_readMulti(struct ToF_struct *ToF, uint8_t reg, uint8_t *dst, uint8_t count)
{
	HAL_I2C_Master_Transmit(&hi2c1, (ToF->bus_address)<<1, reg, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, (ToF->bus_address)<<1,  dst, count, 100);

	dst[0] = 211;
	dst[1] = 0;
	dst[2] = 0;
	dst[3] = 0;
	dst[4] = 0;
	dst[5] = 0;
}

uint8_t ToF_setSignalRateLimit(struct ToF_struct *ToF, float limit_Mcps)
{
	if (limit_Mcps < 0 || limit_Mcps > 511.99) { return 0; }
	// Q9.7 fixed point format (9 integer bits, 7 fractional bits)
	ToF_writeReg16Bit(ToF, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
	return 1;
}

uint8_t ToF_getSpadInfo(struct ToF_struct *ToF, uint8_t *count, uint8_t *type_is_aperture)
{
	uint8_t tmp;

	ToF_writeReg(ToF, 0x80, 0x01);
	ToF_writeReg(ToF, 0xFF, 0x01);
	ToF_writeReg(ToF, 0x00, 0x00);

	ToF_writeReg(ToF, 0xFF, 0x06);
	ToF_writeReg(ToF, 0x83, ToF_readReg(ToF, 0x83) | 0x04);
	ToF_writeReg(ToF, 0xFF, 0x07);
	ToF_writeReg(ToF, 0x81, 0x01);

	ToF_writeReg(ToF, 0x80, 0x01);

	ToF_writeReg(ToF, 0x94, 0x6b);
	ToF_writeReg(ToF, 0x83, 0x00);


	while (ToF_readReg(ToF, 0x83) == 0x00)
	{

	}

	ToF_writeReg(ToF, 0x83, 0x01);
	tmp = ToF_readReg(ToF, 0x92);

	*count = tmp & 0x7f;
	*type_is_aperture = (tmp >> 7) & 0x01;

	ToF_writeReg(ToF, 0x81, 0x00);
	ToF_writeReg(ToF, 0xFF, 0x06);
	ToF_writeReg(ToF, 0x83, ToF_readReg(ToF, 0x83)  & ~0x04);
	ToF_writeReg(ToF, 0xFF, 0x01);
	ToF_writeReg(ToF, 0x00, 0x01);

	ToF_writeReg(ToF, 0xFF, 0x00);
	ToF_writeReg(ToF, 0x80, 0x00);

	return 1;
}

uint16_t ToF_decodeTimeout(struct ToF_struct *ToF, uint16_t reg_val)
{
	  // format: "(LSByte * 2^MSByte) + 1"
	  return (uint16_t)((reg_val & 0x00FF) <<
	         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}
// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.

uint16_t ToF_encodeTimeout(struct ToF_struct *ToF, uint16_t timeout_mclks)
{
	  // format: "(LSByte * 2^MSByte) + 1"

	  uint32_t ls_byte = 0;                     // Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
	  uint16_t ms_byte = 0;                     // based on VL53L0X_calc_timeout_us()

	  if (timeout_mclks > 0)
	  {
	    ls_byte = timeout_mclks - 1;
	    while ((ls_byte & 0xFFFFFF00) > 0)
	    {
	      ls_byte >>= 1;
	      ms_byte++;
	    }
	    return (ms_byte << 8) | (ls_byte & 0xFF);
	  }
	  else { return 0; }
}

uint32_t ToF_timeoutMclksToMicroseconds(struct ToF_struct *ToF, uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
	  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

	  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}
// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()

uint32_t ToF_timeoutMicrosecondsToMclks(struct ToF_struct *ToF, uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
	  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

	  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

void ToF_getSequenceStepEnables(struct ToF_struct *ToF, struct SequenceStepEnables *enables)
{
	uint8_t sequence_config = ToF_readReg(ToF, SYSTEM_SEQUENCE_CONFIG);

	enables->tcc          = (sequence_config >> 4) & 0x1;
	enables->dss          = (sequence_config >> 3) & 0x1;
	enables->msrc         = (sequence_config >> 2) & 0x1;
	enables->pre_range    = (sequence_config >> 6) & 0x1;
	enables->final_range  = (sequence_config >> 7) & 0x1;
}

uint8_t ToF_getVcselPulsePeriod(struct ToF_struct *ToF, enum vcselPeriodType type)
{
	  if (type == VcselPeriodPreRange)
	  {
	    return decodeVcselPeriod(ToF_readReg(ToF, PRE_RANGE_CONFIG_VCSEL_PERIOD));
	  }
	  else if (type == VcselPeriodFinalRange)
	  {
	    return decodeVcselPeriod(ToF_readReg(ToF, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
	  }
	  else { return 255; }
}

void ToF_getSequenceStepTimeouts(struct ToF_struct *ToF, struct SequenceStepEnables *enables, struct SequenceStepTimeouts *timeouts)
{
	timeouts->pre_range_vcsel_period_pclks = ToF_getVcselPulsePeriod(ToF, VcselPeriodPreRange);

		timeouts->msrc_dss_tcc_mclks = ToF_readReg(ToF, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
		timeouts->msrc_dss_tcc_us =
		  ToF_timeoutMclksToMicroseconds(ToF, timeouts->msrc_dss_tcc_mclks,
		                             timeouts->pre_range_vcsel_period_pclks);

		timeouts->pre_range_mclks =
		  ToF_decodeTimeout(ToF, ToF_readReg16Bit(ToF, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
		timeouts->pre_range_us =
			ToF_timeoutMclksToMicroseconds(ToF, timeouts->pre_range_mclks,
		                             timeouts->pre_range_vcsel_period_pclks);

		timeouts->final_range_vcsel_period_pclks = ToF_getVcselPulsePeriod(ToF, VcselPeriodFinalRange);

		timeouts->final_range_mclks =
			ToF_decodeTimeout(ToF, ToF_readReg16Bit(ToF, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

		if (enables->pre_range)
		{
		  timeouts->final_range_mclks -= timeouts->pre_range_mclks;
		}

		timeouts->final_range_us =
			ToF_timeoutMclksToMicroseconds(ToF, timeouts->final_range_mclks,
		                               timeouts->final_range_vcsel_period_pclks);
}

uint32_t ToF_getMeasurementTimingBudget(struct ToF_struct *ToF)
{
	struct SequenceStepEnables enables;
		struct SequenceStepTimeouts timeouts;

		uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
		uint16_t const EndOverhead        = 960;
		uint16_t const MsrcOverhead       = 660;
		uint16_t const TccOverhead        = 590;
		uint16_t const DssOverhead        = 690;
		uint16_t const PreRangeOverhead   = 660;
		uint16_t const FinalRangeOverhead = 550;

		uint32_t budget_us = StartOverhead + EndOverhead;

		ToF_getSequenceStepEnables(ToF, &enables);
		ToF_getSequenceStepTimeouts(ToF, &enables, &timeouts);

		if (enables.tcc)
		{
		  budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
		}
	    if (enables.dss)
		{
		  budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
		}
		else if (enables.msrc)
		{
		  budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
		}

		if (enables.pre_range)
		{
		  budget_us += (timeouts.pre_range_us + PreRangeOverhead);
		}

		if (enables.final_range)
		{
		  budget_us += (timeouts.final_range_us + FinalRangeOverhead);
		}

		ToF->measurement_timing_budget_us = budget_us; // store for internal reuse
		return budget_us;
}

uint8_t ToF_setMeasurementTimingBudget(struct ToF_struct *ToF, uint32_t budget_us)
{
	  struct SequenceStepEnables enables;
	  struct SequenceStepTimeouts timeouts;

	  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
	  uint16_t const EndOverhead        = 960;
	  uint16_t const MsrcOverhead       = 660;
	  uint16_t const TccOverhead        = 590;
	  uint16_t const DssOverhead        = 690;
	  uint16_t const PreRangeOverhead   = 660;
	  uint16_t const FinalRangeOverhead = 550;

	  uint32_t const MinTimingBudget = 20000;

	  if (budget_us < MinTimingBudget) { return 0; }

	  uint32_t used_budget_us = StartOverhead + EndOverhead;

	  ToF_getSequenceStepEnables(ToF, &enables);
	  ToF_getSequenceStepTimeouts(ToF, &enables, &timeouts);

	  if (enables.tcc)
	  {
	    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	  }

	  if (enables.dss)
	  {
	    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	  }
	  else if (enables.msrc)
	  {
	    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	  }

	  if (enables.pre_range)
	  {
	    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	  }

	  if (enables.final_range)
	  {
	    used_budget_us += FinalRangeOverhead;

	    // "Note that the final range timeout is determined by the timing
	    // budget and the sum of all other timeouts within the sequence.
	    // If there is no room for the final range timeout, then an error
	    // will be set. Otherwise the remaining time will be applied to
	    // the final range."

	    if (used_budget_us > budget_us)
	    {
	      // "Requested timeout too big."
	      return 0;
	    }

	    uint32_t final_range_timeout_us = budget_us - used_budget_us;

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

	    // "For the final range timeout, the pre-range timeout
	    //  must be added. To do this both final and pre-range
	    //  timeouts must be expressed in macro periods MClks
	    //  because they have different vcsel periods."

	    uint16_t final_range_timeout_mclks =
	      ToF_timeoutMicrosecondsToMclks(ToF, final_range_timeout_us,
	                                 timeouts.final_range_vcsel_period_pclks);

	    if (enables.pre_range)
	    {
	      final_range_timeout_mclks += timeouts.pre_range_mclks;
	    }

	    ToF_writeReg16Bit(ToF, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
	      ToF_encodeTimeout(ToF, final_range_timeout_mclks));

	    // set_sequence_step_timeout() end

	    ToF->measurement_timing_budget_us = budget_us; // store for internal reuse
	  }
	  return 1;
}

uint8_t ToF_performSingleRefCalibration(struct ToF_struct *ToF, uint8_t vhv_init_byte)
{
	ToF_writeReg(ToF, SYSRANGE_START, 0x01 | vhv_init_byte);

	  while ((ToF_readReg(ToF, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
	  {
	    //timeout
	  }

	  ToF_writeReg(ToF, SYSTEM_INTERRUPT_CLEAR, 0x01);

	  ToF_writeReg(ToF, SYSRANGE_START, 0x00);

	  return 1;
}

uint8_t ToF_init(struct ToF_struct *ToF)
{
	ToF->bus_address = 0x29;


    ToF_writeReg(ToF, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, ToF_readReg(ToF, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0

	ToF_writeReg(ToF, 0x88, 0x00);

	ToF_writeReg(ToF, 0x80, 0x01);
	ToF_writeReg(ToF, 0xFF, 0x01);
	ToF_writeReg(ToF, 0x00, 0x00);
	ToF->stop_variable = ToF_readReg(ToF, 0x91);
	ToF_writeReg(ToF, 0x00, 0x01);
	ToF_writeReg(ToF, 0xFF, 0x00);
	ToF_writeReg(ToF, 0x80, 0x00);

	// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	ToF_writeReg(ToF, MSRC_CONFIG_CONTROL, ToF_readReg(ToF, MSRC_CONFIG_CONTROL) | 0x12);

	// set final range signal rate limit to 0.25 MCPS (million counts per second)
	ToF_setSignalRateLimit(ToF, 0.25);

	ToF_writeReg(ToF, SYSTEM_SEQUENCE_CONFIG, 0xFF);
	uint8_t spad_count;
	uint8_t spad_type_is_aperture;

	if (ToF_getSpadInfo(ToF, &spad_count, &spad_type_is_aperture) == 0)
	{
		return 0;
	}

	uint8_t ref_spad_map[6];
	ToF_readMulti(ToF, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	ToF_writeReg(ToF, 0xFF, 0x01);
	ToF_writeReg(ToF, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	ToF_writeReg(ToF, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	ToF_writeReg(ToF, 0xFF, 0x00);
	ToF_writeReg(ToF, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

	uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
	uint8_t spads_enabled = 0;

	for (uint8_t i = 0; i < 48; i++)
	{
	  if (i < first_spad_to_enable || spads_enabled == spad_count)
	  {
	    // This bit is lower than the first one that should be enabled, or
	    // (reference_spad_count) bits have already been enabled, so zero this bit
	    ref_spad_map[i / 8] &= ~(1 << (i % 8));
	  }
	  else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
	  {
		  spads_enabled++;
	  }
	}
	ToF_writeMulti(ToF, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	ToF_writeReg(ToF, 0xFF, 0x01);
	ToF_writeReg(ToF, 0x00, 0x00);

	ToF_writeReg(ToF, 0xFF, 0x00);
	ToF_writeReg(ToF, 0x09, 0x00);
	ToF_writeReg(ToF, 0x10, 0x00);
	ToF_writeReg(ToF, 0x11, 0x00);

	ToF_writeReg(ToF, 0x24, 0x01);
	ToF_writeReg(ToF, 0x25, 0xFF);
	ToF_writeReg(ToF, 0x75, 0x00);

	ToF_writeReg(ToF, 0xFF, 0x01);
	ToF_writeReg(ToF, 0x4E, 0x2C);
	ToF_writeReg(ToF, 0x48, 0x00);
	ToF_writeReg(ToF, 0x30, 0x20);

	ToF_writeReg(ToF, 0xFF, 0x00);
	ToF_writeReg(ToF, 0x30, 0x09);
	ToF_writeReg(ToF, 0x54, 0x00);
	ToF_writeReg(ToF, 0x31, 0x04);
	ToF_writeReg(ToF, 0x32, 0x03);
	ToF_writeReg(ToF, 0x40, 0x83);
	ToF_writeReg(ToF, 0x46, 0x25);
	ToF_writeReg(ToF, 0x60, 0x00);
	ToF_writeReg(ToF, 0x27, 0x00);
	ToF_writeReg(ToF, 0x50, 0x06);
	ToF_writeReg(ToF, 0x51, 0x00);
	ToF_writeReg(ToF, 0x52, 0x96);
	ToF_writeReg(ToF, 0x56, 0x08);
	ToF_writeReg(ToF, 0x57, 0x30);
	ToF_writeReg(ToF, 0x61, 0x00);
	ToF_writeReg(ToF, 0x62, 0x00);
	ToF_writeReg(ToF, 0x64, 0x00);
	ToF_writeReg(ToF, 0x65, 0x00);
	ToF_writeReg(ToF, 0x66, 0xA0);

	ToF_writeReg(ToF, 0xFF, 0x01);
	ToF_writeReg(ToF, 0x22, 0x32);
	ToF_writeReg(ToF, 0x47, 0x14);
	ToF_writeReg(ToF, 0x49, 0xFF);
	ToF_writeReg(ToF, 0x4A, 0x00);

	ToF_writeReg(ToF, 0xFF, 0x00);
	ToF_writeReg(ToF, 0x7A, 0x0A);
	ToF_writeReg(ToF, 0x7B, 0x00);
	ToF_writeReg(ToF, 0x78, 0x21);

	ToF_writeReg(ToF, 0xFF, 0x01);
	ToF_writeReg(ToF, 0x23, 0x34);
	ToF_writeReg(ToF, 0x42, 0x00);
	ToF_writeReg(ToF, 0x44, 0xFF);
	ToF_writeReg(ToF, 0x45, 0x26);
	ToF_writeReg(ToF, 0x46, 0x05);
	ToF_writeReg(ToF, 0x40, 0x40);
	ToF_writeReg(ToF, 0x0E, 0x06);
	ToF_writeReg(ToF, 0x20, 0x1A);
	ToF_writeReg(ToF, 0x43, 0x40);

	ToF_writeReg(ToF, 0xFF, 0x00);
	ToF_writeReg(ToF, 0x34, 0x03);
	ToF_writeReg(ToF, 0x35, 0x44);

	ToF_writeReg(ToF, 0xFF, 0x01);
	ToF_writeReg(ToF, 0x31, 0x04);
	ToF_writeReg(ToF, 0x4B, 0x09);
	ToF_writeReg(ToF, 0x4C, 0x05);
	ToF_writeReg(ToF, 0x4D, 0x04);

	ToF_writeReg(ToF, 0xFF, 0x00);
	ToF_writeReg(ToF, 0x44, 0x00);
	ToF_writeReg(ToF, 0x45, 0x20);
	ToF_writeReg(ToF, 0x47, 0x08);
	ToF_writeReg(ToF, 0x48, 0x28);
	ToF_writeReg(ToF, 0x67, 0x00);
	ToF_writeReg(ToF, 0x70, 0x04);
	ToF_writeReg(ToF, 0x71, 0x01);
	ToF_writeReg(ToF, 0x72, 0xFE);
	ToF_writeReg(ToF, 0x76, 0x00);
	ToF_writeReg(ToF, 0x77, 0x00);

	ToF_writeReg(ToF, 0xFF, 0x01);
	ToF_writeReg(ToF, 0x0D, 0x01);

	ToF_writeReg(ToF, 0xFF, 0x00);
	ToF_writeReg(ToF, 0x80, 0x01);
	ToF_writeReg(ToF, 0x01, 0xF8);

	ToF_writeReg(ToF, 0xFF, 0x01);
	ToF_writeReg(ToF, 0x8E, 0x01);
	ToF_writeReg(ToF, 0x00, 0x01);
	ToF_writeReg(ToF, 0xFF, 0x00);
	ToF_writeReg(ToF, 0x80, 0x00);

	ToF_writeReg(ToF, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
	ToF_writeReg(ToF, GPIO_HV_MUX_ACTIVE_HIGH, ToF_readReg(ToF, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
	ToF_writeReg(ToF, SYSTEM_INTERRUPT_CLEAR, 0x01);

	ToF->measurement_timing_budget_us = ToF_getMeasurementTimingBudget(ToF);

	ToF_writeReg(ToF, SYSTEM_SEQUENCE_CONFIG, 0xE8);

	ToF_setMeasurementTimingBudget(ToF, ToF->measurement_timing_budget_us);

	ToF_writeReg(ToF, SYSTEM_SEQUENCE_CONFIG, 0x01);

	if (!ToF_performSingleRefCalibration(ToF, 0x40)) { return 0; }

	ToF_writeReg(ToF, SYSTEM_SEQUENCE_CONFIG, 0x02);

    if (!ToF_performSingleRefCalibration(ToF, 0x00)) { return 0; }

    ToF_writeReg(ToF, SYSTEM_SEQUENCE_CONFIG, 0xE8);

	return 1;
}

void ToF_setAddress(struct ToF_struct *ToF, uint8_t new_addr)
{
	ToF_writeReg(ToF, I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
	ToF->bus_address = new_addr;
}

void ToF_startContinuous(struct ToF_struct *ToF)
{
	ToF_writeReg(ToF, 0x80, 0x01);
	ToF_writeReg(ToF, 0xFF, 0x01);
	ToF_writeReg(ToF, 0x00, 0x00);
	ToF_writeReg(ToF, 0x91, ToF->stop_variable);
	ToF_writeReg(ToF, 0x00, 0x01);
	ToF_writeReg(ToF, 0xFF, 0x00);
	ToF_writeReg(ToF, 0x80, 0x00);

	ToF_writeReg(ToF, SYSRANGE_START, 0x02);
}

uint16_t ToF_readRangeContinuousMillimeters(struct ToF_struct *ToF)
{
	while ((ToF_readReg(ToF,RESULT_INTERRUPT_STATUS) & 0x07) == 0)
	{
		//timeout
	}

	uint16_t range = ToF_readReg16Bit(ToF, RESULT_RANGE_STATUS + 10);
	ToF_writeReg(ToF, SYSTEM_INTERRUPT_CLEAR, 0x01);
	return range;
}
uint16_t ToF_readRangeSingleMillimeters(struct ToF_struct *ToF)
{
	ToF_writeReg(ToF, 0x80, 0x01);
	ToF_writeReg(ToF, 0xFF, 0x01);
	ToF_writeReg(ToF, 0x00, 0x00);
	ToF_writeReg(ToF, 0x91, ToF->stop_variable);
	ToF_writeReg(ToF, 0x00, 0x01);
	ToF_writeReg(ToF, 0xFF, 0x00);
	ToF_writeReg(ToF, 0x80, 0x00);

	ToF_writeReg(ToF, SYSRANGE_START, 0x01);

	while((ToF_readReg(ToF, SYSRANGE_START) & 0x01) == 1)
	{
		//timeout
	}
	return ToF_readRangeContinuousMillimeters(ToF);
}



/* funkcje i struktury specyficzne dla micromouse
 *
 * Poniższe funkcje i struktury mają zastosowanie tylko w przypadku micromouse i mają służyć do wyczyszczenia
 * funkcji main z nadmiaru kodu.
 *
 */

void initMicromouseVL53L0x()
{
	  //////////////////////////////////////////////////////////////////////////
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_F_Pin, 0);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_FR_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_FL_Pin, 0);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_R_Pin, 0);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_L_Pin, 0);

	  ToF_init(&ToF_FR);                      //inicjalizacja czujnika FR
	  ToF_setAddress(&ToF_FR, 0x30);

	  //////////////////////////////////////////////////////////////////////////
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_F_Pin, 0);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_FR_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_FL_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_R_Pin, 0);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_L_Pin, 0);

	  ToF_init(&ToF_FL);                      //inicjalizacja czujnika FL
	  ToF_setAddress(&ToF_FL, 0x31);

	  //////////////////////////////////////////////////////////////////////////
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_F_Pin, 0);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_FR_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_FL_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_R_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_L_Pin, 0);

	  ToF_init(&ToF_R);                     //inicjalizacja czujnika R
	  ToF_setAddress(&ToF_R, 0x32);

	  //////////////////////////////////////////////////////////////////////////
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_F_Pin, 0);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_FR_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_FL_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_R_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_L_Pin, 1);

	  ToF_init(&ToF_L);                     //inicjalizacja czujnika L
	  ToF_setAddress(&ToF_L, 0x33);

	  //////////////////////////////////////////////////////////////////////////
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_F_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_FR_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_FL_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_R_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_L_Pin, 1);

	  ToF_init(&ToF_F);                     //inicjalizacja czujnika F
	  ToF_setAddress(&ToF_F, 0x34);
	  //////////////////////////////////////////////////////////////////////////

	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_F_Pin, 1);     //uruchomienie wszystkich ToF
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_FR_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_FL_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_R_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, TOF_GPIO_L_Pin, 1);

	  ToF_startContinuous(&ToF_F);                     //uruchomienie trybu pomiaru ciągłego
	  ToF_startContinuous(&ToF_R);
	  ToF_startContinuous(&ToF_L);
	  ToF_startContinuous(&ToF_FR);
	  ToF_startContinuous(&ToF_FL);
}

