#
# Drop-in replacement for:
#   FreeRTOS/Demo/R-Car_Gen5_CR52/sample_apps/dummy_app
#
# It injects trace hook macros into freertos_bsp at compile time, so no manual
# edits to the BSP's FreeRTOSConfig.h are required.
#

set(app_name "trace_pipeline_app")

set(RCAR_TRACE_CAPTURE_MS 2000 CACHE STRING "Trace capture duration in milliseconds")
set(RCAR_TRACE_BUFFER_EVENTS 65536 CACHE STRING "Trace ring buffer capacity in events")

set(trace_force_include "SHELL:-include ${CMAKE_CURRENT_LIST_DIR}/trace_port_rcar_gen5.h")
set(trace_common_defs
    RCAR_TRACE_CAPTURE_MS=${RCAR_TRACE_CAPTURE_MS}
    RCAR_TRACE_BUFFER_EVENTS=${RCAR_TRACE_BUFFER_EVENTS}
)

if(DEFINED RCAR_TRACE_BUFFER_BASE)
    list(APPEND trace_common_defs RCAR_TRACE_BUFFER_BASE=${RCAR_TRACE_BUFFER_BASE})
endif()

add_executable(${app_name}
    main.c
    trace_freertos.c
)

target_include_directories(freertos_bsp PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_options(freertos_bsp PRIVATE
    ${trace_force_include}
)

target_compile_options(${app_name} PRIVATE
    ${trace_force_include}
)

target_compile_definitions(freertos_bsp PUBLIC
    ${trace_common_defs}
)

target_compile_definitions(${app_name} PUBLIC
    ${trace_common_defs}
)

target_link_libraries(${app_name}
    freertos_bsp m
)

target_link_options(${app_name}
    PUBLIC
        -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${app_name}.map
)

convert_bin_to_srec(${app_name})
