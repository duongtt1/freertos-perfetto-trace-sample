# Renesas R-Car Gen5 CR52 Trace Perfetto App

This folder contains a drop-in replacement for:

`FreeRTOS/Demo/R-Car_Gen5_CR52/sample_apps/freertos-perfetto-trace-sample`

## Quick Use

1. In the Renesas repo/branch:
   `renesas-rcar/FreeRTOS.git`
   `rcar_freertos_bsp_ironhide`

2. Copy this folder to:
   `FreeRTOS/Demo/R-Car_Gen5_CR52/sample_apps/freertos-perfetto-trace-sample`

3. Modify the parent `CMakeLists.txt` to include this sample app:

```cmake
add_subdirectory(sample_apps/freertos-perfetto-trace-sample)
```
4. Build as usual

## What This App Does

- Creates a queue-driven pipeline:
  `Cam -> Dispatcher -> VIN -> ISP -> IMR -> VSPD`
- Models:
  - 4 cameras
  - 4 VIN workers
  - 2 ISP workers
  - 10 IMR workers
  - 4 VSPD workers
- Emits scheduler task slices, queue events, marker events, and ISR enter/exit
  events for Perfetto conversion.

## Trace Integration Model

The app forces a local config shim into the BSP compile:

- `trace_port_rcar_gen5.h`
- `trace_freertos.h`
- `trace_freertos.c`

That means you do not need to manually edit the BSP's global
`include/FreeRTOSConfig.h` just to enable trace hooks.

## Buffer Placement

By default the trace ring buffer is allocated internally.

To place the buffer at a fixed address, pass a build definition such as:

- `RCAR_TRACE_BUFFER_BASE=0x68000000`
- `RCAR_TRACE_BUFFER_EVENTS=524288`

The shim maps those to:

- `TRACE_RING_BUFFER_PTR`
- `TRACE_RING_BUFFER_CAPACITY`

## Important Files

- `dummy_app/CMakeLists.txt`
  Injects the trace config into `freertos_bsp` using force-include.
- `dummy_app/trace_port_rcar_gen5.h`
  CR52-specific trace defaults and hook macro bridge.
- `dummy_app/trace_freertos.h`
- `dummy_app/trace_freertos.c`
- `dummy_app/main.c`

## Host Conversion

Use the converter from this repo after capturing UART logs:

`tools/uart_to_perfetto.py`

It expects the UART dump markers:

- `#UART_TRACE_BEGIN`
- `#UART_TRACE_END`
