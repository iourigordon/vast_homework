#include <stdint.h>
#include <sys/ioctl.h>

#include <string>
#include <vector>
#include <stdexcept>

#include "device.h"
#include "adc.h"

struct ads114_config {
    uint8_t registers[16];  // ADS114S08B has 16 configuration registers
};

#define IOCTL_SET_REGS  _IOW('a', 1, uint16_t*)  // Set ADC registers
#define IOCTL_GET_REGS  _IOR('a', 2, uint8_t*)  // Get ADC registers

#define VREF 2.5  // Reference voltage in volts
#define ADC_RESOLUTION 32768.0  // 16-bit signed ADC (±32768)
#define PGA_GAIN 1  // Programmable Gain Amplifier (Adjust as needed)

int
adc::readData(std::vector<double>& samples)
{
	uint16_t sample;
	double val;
	if (read(m_fd,&sample,sizeof(uint16_t)) == -1) {
		throw std::runtime_error("failed to read reg");
	}

	/*
		convert to double here, using defines from above
		in real world just read registers and get all the values
		from them
	*/
	val = ((double)sample / ADC_RESOLUTION) * (VREF / PGA_GAIN);
	return 0;
}

int
adc::initRegsCache()
{
	uint8_t reg_read;
	for (int i=0; i<REG_MAX; i++) {
		m_reg_cache[i] = readReg(i);
	}
	return 0;
}

int
adc::setChannel(int pos_ch, int neg_ch)
{
	/* set positive/negative ch bits */
	uint8_t val = ((pos_ch&0x0F)<<4)|(neg_ch&0x0F);
	return writeReg(REG_MUX_REG,
}

int
adc::writeReg(int reg, uint8_t val)
{
	uint16_t reg_write;
	reg_write = ((0x40|reg)<<8)|val ;
	if (ioctl(m_fd,IOCTL_SET_REGS,&reg_write) == -1) {
		throw std::runtime_error("failed to read reg");
	}
	return 0;
}

uint8_t
adc::readReg(int reg)
{
	uint8_t reg_read;
	reg_read = 0x20|reg;
	if (ioctl(m_fd,IOCTL_GET_REGS,&reg_read) == -1) {
		throw std::runtime_error("failed to read reg");
	}
	return reg_read;
}
