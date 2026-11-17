# STM32F429I-DISC1 project ideas

Roughly easiest to hardest.

1. **SDRAM bring-up and memory test.** Set up the FMC, then do walking-ones, address-line and throughput tests. Show the results on the LCD and over UART. Everything else depends on this, so it's the right first project.
2. **Double-buffered graphics with LTDC and DMA2D.** Use two framebuffers in SDRAM and swap them on vsync. Use DMA2D for fills, blits and alpha blending. You could build a small 2D demo or an original game on top, such as a tilt-controlled maze with the gyro as input.
3. **Air-quality dashboard.** This reuses drivers you already have in the repo: SEN55, SGP40, PM2.5, BME280 and BMP280. Store hours of samples in SDRAM as ring buffers and draw scrolling graphs on the LCD. A single 1 Hz channel stored for 24 hours takes about 345 KB, which is trivial for SDRAM and impossible in internal RAM. This is the most practical project for your setup.
4. **Gyro attitude visualizer.** Integrate the on-board gyro, or fuse it with your external IMU, and draw an artificial horizon or a rotating 3D wireframe. It's a good way to learn fixed-point or FPU math plus rendering.
6. **ArduCAM viewer (from your TODO).** Read JPEG frames from the SPI camera into SDRAM, decode them with TJpgDec, and show them on the LCD. The 5 MP frames can't fit anywhere except SDRAM. Most of the DCMI pins are taken by the LCD on this board, so SPI is the right interface.
7. **USB data logger.** Collect sensor data in SDRAM and expose it as a USB MSC drive or CDC stream, or use host mode to write CSV files to a USB stick.

## In progress

- 3 + 7 combined: [`001_aq_logger`](001_aq_logger/docs/SPEC.md) (SEN55 + STCC4, LCD dashboard, CSV to USB stick).
