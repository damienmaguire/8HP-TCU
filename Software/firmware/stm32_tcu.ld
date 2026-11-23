/* Linker script for the 8HP TCU on an STM32F107VCT6 (256 KiB flash, 64 KiB RAM).
 *
 * 0x08000000  4 KiB  bootloader (openinverter stm32-sine style)
 * 0x08001000  244 KiB application (this image)
 * 0x0803E000  4 KiB  spare
 * 0x0803F000  2 KiB  CAN map (libopeninv: end of flash - 2 pages)
 * 0x0803F800  2 KiB  parameters (libopeninv: last page)
 */
MEMORY
{
	rom (rx)    : ORIGIN = 0x08001000, LENGTH = 244K
	ram (rwx)   : ORIGIN = 0x20000000, LENGTH = 64K
}

/* Include the common ld script from libopencm3. */
INCLUDE cortex-m-generic.ld
