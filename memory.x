/* ABOUTME: Defines the STM32F446RE flash and RAM regions for the embedded linker.
 * ABOUTME: Provides the memory map used by the Cortex-M runtime for the firmware image.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
