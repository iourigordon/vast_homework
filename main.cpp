#include <stdint.h>

#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>

#include "device.h"
#include "adc.h"

int main(int argc, char* argv[])
{
	try {
		adc adc_114("pru_adc");
	}
	catch (const std::exception& e) {
		std::cout << "hehe" << std::endl;
	}
	std::cout << "Hello" << std::endl;
}
