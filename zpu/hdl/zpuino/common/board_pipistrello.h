#ifndef __BOARD_H__
#define __BOARD_H__

#define CLK_FREQ 100000000UL

/* LX45 bitfile is 0x16A7EE in size */

#define SPIOFFSET 0x170000

#ifndef BOARD_MEMORYSIZE
#error Undefined board memory size
#endif

#define BOARD_SPI_DIVIDER BIT(SPICP0)

#define IOBASE 0x08000000
#define IO_SLOT_OFFSET_BIT 23

#define FPGA_PIN_FLASHCS     48
#define FPGA_PIN_LED1        49
#define FPGA_PIN_LED2        50
#define FPGA_PIN_LED3        51
#define FPGA_PIN_LED4        52

#define SPI_FLASH_SEL_PIN FPGA_PIN_FLASHCS

#endif
