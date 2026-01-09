(1) We use QEMU version 8.2 in performance test.
(2) To confirm that unit tests can be run smoothly, we set the RAM/FLASH of board in QEMU as 8MB, 
	More Details: qemu-8.2/include/hw/arm/stm32f405_soc.h (FLASH Size/ SRAM_SIZE, line 47-49)
(3) Besides, because there is one cases in zip unit test need too much ram that exceed the valid range of MCU.
	So, we just remove that cases.
	More Details: coremark-pro/benchmarks/darkmark/zip/zip_darkmark.c (remove "dataset 0" in zip_params, line 78)
