#include <fcntl.h>
#include <unistd.h>

#include <string>
#include <vector>
#include <stdexcept>

#include "device.h"

device::device(std::string dev_name): m_dev_name(dev_name)
{
	m_fd = open(m_dev_name.c_str(),O_RDWR);
	if (m_fd == -1) {
		throw std::runtime_error("Something went wrong!");
	}
}

device::~device()
{
	close(m_fd);
}
