class adc : public device {
	public:

		enum {
			REG_DEV_ID,
			REG_DEV_STATUS,
			REG_MUX_REG,
			REG_GAIN,
			REG_REF_CTRL,
			REG_EXCIT_1,
			REG_EXCIT_2,
			REG_SENSOR_BIAS,
			REG_SYS_CTRL,
			REG_RESERVED_1,
			REG_OFFSET_CAL_1,
			REG_OFFSET_CAL_2,
			REG_RESERVED_2,
			REG_GAIN_CAL_1,
			REG_GAIN_CAL_2,
			REG_GPIO_DATA,
			REG_GPIO_CONFIG,
			REG_MAX
		};

		adc(std::string dev_name):device(dev_name) {}
		~adc() {}

		int readData(std::vector<double>& samples) override;
		int initRegsCache() override;
		int writeReg() override;
		int readReg() override;


		int setChannel(int ch);

	private:
		uint8_t m_reg_cache[REG_MAX];
};
