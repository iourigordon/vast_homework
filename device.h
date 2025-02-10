class device {
	public:
		device(std::string dev_name);
		~device();

		virtual int readData(std::vector<double>& samples) = 0;
		virtual int initRegsCache() = 0;
		virtual int writeReg() = 0;
		virtual int readReg()  = 0;

	protected:
		std::string m_dev_name;
		int	m_fd;
};
