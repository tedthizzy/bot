# firmware/core/ is freestanding C11 with zero IDF headers and is owned by
# another component. It is compiled into this one rather than made an IDF
# component, so nothing in firmware/core/ needs a build file of its own and
# mcu-sim keeps linking the same sources through its ctypes shim.
file(GLOB ROVER_CORE_SRCS "${CMAKE_CURRENT_LIST_DIR}/../core/*.c")

idf_component_register(
    SRCS "app_main.c"
         "link.c"
         "motion.c"
         "power.c"
         "sensors.c"
         "vl53l4cx.c"
         ${ROVER_CORE_SRCS}
    INCLUDE_DIRS "."
                 "${CMAKE_CURRENT_LIST_DIR}/../core/include"
                 "${CMAKE_CURRENT_LIST_DIR}/../core"
    REQUIRES driver esp_adc esp_timer
)

# The core is built to the same standard it is tested to on the host.
set_source_files_properties(${ROVER_CORE_SRCS} PROPERTIES COMPILE_OPTIONS "-std=c11")
