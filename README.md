# vast_homework

Blackberry Black was taken as a reference platform to solve Vast homework. 
It provides ARM Cpu with 2 PRU microcontrollers, spi controller abundance of GPIO pin.

## CPU
### UserLand
sample C++ code is provided in 4 files: adc.cpp/h and device.cpp/h. 
device.cpp/h define an interface that adc should implement. adc.cpp/h provides actual implementation of controlling adc

### Kernel Driver
Kernel mechanism provides a mechanism to communicate to the application running on PRU.
It provides ioctl control over adc registers, and read call to retreive adc samples

Kernel driver ioctl and read are blocking, waiting for the interrupt from PRU to complete.

## PRU
Runs baremetal code. SPI peripheral is mapped into PRU memory and configured to be a master. ADC pins should be connected to
PRU SPI as described in Beaglebone Black documentation. (Use SPI0 pins)

### Sample retrieval
Once SPI perpheral is initialized application check for commands and read data from adc one sample at a time using RDATA command.
When data is read out it is stored in a circular buffer located in shared memory. GPIO interrupt is raised to 
notify CPU of available data. This mechanism works only with small data rates, moving to higher ones will require DMA based
implementation.

### Command Execution

Pru code checks shared memory command location, and when command is available it sends it over to adc. Once transaction is completed, PRU raises GPIO interrupt to notify CPU of completion.

## Memory
There is a shared memory between PRU and and ARM Cpu. Circular buffer used to store adc samples is located at the beginning of shared memory, followed by command and value.



