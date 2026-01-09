#### **Configure QEMU**

(1) We use  QEMU 8.2.10 in the performance test. To confirm that unit tests can be run smoothly, we set the RAM/FLASH of the board in QEMU to 8MB. 

(2) Besides, there is one test item in the zip unit test that needs too much RAM, which exceeds the valid range of the MCU. So, we just remove those test items.  More Details: coremark-pro/benchmarks/darkmark/zip/zip_darkmark.c (remove "dataset 0" in zip_params, line 78)
