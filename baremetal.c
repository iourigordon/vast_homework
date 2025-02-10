#include <stdint.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <pru_spi.h>
#include "resource_table.h"

/*
	__R31,__R30 is mapped to PRU, and can be used to toogle GPIO pins
*/
volatile register uint32_t __R31;
volatile register uint32_t __R30;

/*
for spi controller base refer to:
AM335x_Tech_Ref_Manual.pdf
table 2-3
*/
#define SPI_BASE_ADDR    0x48030000

/*
for spi regs offsets refer to:
AM335x_Tech_Ref_Manual.pdf
table 24-10
*/
volatile uint32_t* spi_sysc_r		= SPI_BASE_ADDR + 0x110;
volatile uint32_t* spi_modulctrl_r	= SPI_BASE_ADDR + 0x128;
volatile uint32_t* spi_ch0conf_r    = SPI_BASE_ADDR + 0x12C;
volatile uint32_t* spi_ch0ctrl_r	= SPI_BASE_ADDR + 0x134;
volatile uint32_t* spi_tx_r			= SPI_BASE_ADDR + 0x138;
volatile uint32_t* spi_rx_r			= SPI_BASE_ADDR + 0x13C;
volatile uint32_t* spi_ch0status_r	= SPI_BASE_ADDR + 0x130;

#define SPI_TX_EMPTY (1<<1)
#define SPI_RX_FULL  (1<<0)

/*
toggle chip select GPIO5
*/
#define CS_LOW          __R30 &= ~(1 << 3)
#define CS_HIGH         __R30 |=  (1 << 3)

//#define SPI_IRQENABLE    *(volatile uint32_t *)(SPI_BASE_ADDR + 0x11C)

/* GPIO for DRDY Pin (Assuming it's mapped to a PRU GPIO pin 14) */
#define GPIO_BASE_ADDR   0x44E07000
#define GPIO_DATAIN      *(volatile uint32_t *)(GPIO_BASE_ADDR + 0x138)
#define DRDY_PIN         (1 << 14)  // Adjust based on connection

/*
	 PRU/ARM shared Memory
*/
#define PRU_SHARED_MEM   ((volatile uint32_t *) 0x00010000)
#define BUFFER_SIZE 0x400

#define BUFFER_PRIME_ADDRESS		PRU_SHARED_MEM
/*
	 ADS114S0xB Read/Write
*/
#define CMD_READ_REG     0x20	/* Read Register Command */
#define CMD_WRITE_REG    0x40	/* Write Register Command */
#define CMD_RDATA_REG	 0x12	/* Read Data Command */

/*
	setup spi interface to SPI mode 1,
	as it is the only mode supported by ads114S08B
*/

typedef struct {
    uint16_t data[BUFFER_SIZE];
    uint32_t head;
    uint32_t tail;

    uint8_t  command; /* only read and write register cmd and addr*/
    uint8_t  value;
} CircularBuffer;

volatile CircularBuffer *buffer = (CircularBuffer *) BUFFER_PRIME_ADDRESS;

void spi_init() {
    /* Reset SPI module */
    *spi_sysc_r = 0x2;  /* Trigger Reset */
    while (*spi_sysc_r & 0x2);  /* Wait for reset to complete*/

    /* Set SPI Mode: Master Mode, Mode 1 (CPOL=0, CPHA=1) */
    *mdulctrl_r = 0x0;  // Disable single-channel mode
    *spi_ch0conf_r = (1 << 6)	/* CPHA = 1 (Mode 1) */
                 | (0 << 7)		/* CPOL = 0 */
                 | (1 << 20)	/* 16-bit word length */
                 | (1 << 18)	/* Force chip select active */
                 | (0 << 25);	/* SPI master mode */

    // Enable SPI Channel
    *spi_ch0ctrl_r = 1;
}

static inline uint8_t spi_transfer(uint8_t data) {
	/* check that tx register is empty */
    while (!(*spi_ch0status_r & SPI_TX_EMPTY));
	*spi_tx_r = tx_data;

	/* wait for spi data to comeout */
    while (!(*spi_ch0status_r & SPI_RX_FULL));
    return *spi_rx_r;  // Read received data
}

void write_register(uint8_t reg, uint8_t value) {
	CS_LOW;
    spi_transfer(CMD_WRITE_REG | reg);  // Send Write Register Command
    spi_transfer(0x00);  // Number of bytes (1)
    spi_transfer(value);  // Write data
    CS_HIGH;
}

uint8_t read_register(uint8_t reg) {
	uint8_t r_data;

	CS_LOW;
    spi_transfer(CMD_READ_REG | reg);
    spi_transfer(0x00);
	r_data = spi_transfer(0x00);
    CS_HIGH;

    return r_data;
}

uint16_t read_adc_spi(void) {
    uint8_t high_byte, low_byte;
    uint16_t adc_value;

    CS_LOW;
    delay_cycles(10);

    spi_transfer(CMD_RDATA_REG);
    delay_cycles(10);

    high_byte = spi_transfer(0x00);  // Read high byte
    low_byte = spi_transfer(0x00);   // Read low byte

    CS_HIGH;

    // Combine two bytes into a 16-bit value
    adc_value = ((uint16_t)high_byte << 8) | low_byte;

    return adc_value;
}

void main() {

	volatile CircularBuffer *current = buffer_1;

    spi_init();

	buffer->command = 0;
	buffer->value	= 0;

    while (1) {
		if (buffer->command != 0) {
			if (buffer->command & CMD_READ_REG) {
				buffer->val = read_register(buffer->command);
			}
			if (buffer->command & CMD_WRITE_REG) {
				write_register(buffer->command, buffer->value);
			}
			/*signal cpu we are down with regs*/
			buffer->command = 0;
			__R31 = (1 << 7) | PRU_INTR_NUMBER;
		}
        if (__R31 & (1 << 15)) { /* Check DRDY pin for ADC data ready */
            uint16_t adc_value = read_adc_spi(); // Implement SPI read function

            uint32_t next = (current->head + 1) % BUFFER_SIZE;
            uint32_t next = (buffer->head + 1) % BUFFER_SIZE;
            if (next != buffer->tail) {
                buffer->data[buffer->head] = adc_value;
                buffer->head = next;
            }

			/*signal cpu data is ready*/
		    __R31 = (1 << 5) | PRU_INTR_NUMBER;
        }
    }

    while (1) {
        // Wait for DRDY to go LOW
        while (GPIO_DATAIN & DRDY_PIN);

        // Read ADC Data
        uint16_t adc_data = spi_transfer(0x12);  // Example command to read ADC

        // Store ADC data in PRU shared memory
        PRU_SHARED_MEM[0] = adc_data;
    }
}

