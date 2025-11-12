# Find Zephyr. This also loads Zephyr's build system.
cmake_minimum_required(VERSION 3.26.3)
if(NOT DEFINED BOARD)
  set(BOARD oxycontroller4_samd21)
endif()
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_zephyr_app)

set(MICROLIB_DIR "../deps/microlib2")
set(MICROLIB_ARCH "zephyr")
include("${MICROLIB_DIR}/CMakeLists.txt")

# Get the current version from git
execute_process(COMMAND git describe
        #WORKING_DIRECTORY "${MICROLIB}"
        OUTPUT_VARIABLE "PROJECT_VERSION_STRING")
string(STRIP ${PROJECT_VERSION_STRING} PROJECT_VERSION_STRING)

# This should give the includes, options and defines from Zephyr
# to be used to compile our external libraries, but for some
# reason they have some quotes which make the variables not
# usable.
zephyr_get_include_directories_for_lang(       C includes STRIP_PREFIX)
zephyr_get_system_include_directories_for_lang_as_string(C system_includes STRIP_PREFIX)
zephyr_get_compile_definitions_for_lang(       C definitions STRIP_PREFIX)
zephyr_get_compile_options_for_lang(C options_xxx STRIP_PREFIX)
set(options "SHELL: ${options_xxx}")

# Show the info about our project
message("Project: ${PROJECT_NAME}")
message("Microlib acrhitecture ${MICROLIB_ARCH}")
message("Microlib version: ${MICROLIB_VERSION_STRING}")
message("Project version: ${PROJECT_VERSION_STRING}")
message("BSP include dir: ${BSP_INCLUDE}")
message("Board: ${BOARD}")
message("Options: ${options_xxx}")
message("Includes: ${includes}")
message("Definitions: ${definitions}")
message("SNIPPET_ROOT is set to: ${SNIPPET_ROOT}")


# Generate version file
configure_file(src/version.h.in version.h)

#add_library(driverlib STATIC
#    ${MICROLIB_ARCH_DIR}/serial.c
#)

# For some reason, fortify is not working correctly on MacOS
zephyr_compile_options(-U_FORTIFY_SOURCE)

list(REMOVE_ITEM MICROLIB_SOURCES 
    ${MICROLIB_SRC}/temperature_thermistor.c
    ${MICROLIB_SRC}/circular_object_storage.c
    ${MICROLIB_SRC}/luminescence_sensor.c
    ${MICROLIB_SRC}/oxygen_optic_ui.c
    )
add_library(microlib STATIC ${MICROLIB_SOURCES})

target_include_directories(microlib PUBLIC "${MICROLIB_INCLUDE_DIR}" "${BOARD_INCLUDE}" "${PROJECT_BINARY_DIR}" src)
# Options for compiling the microlib. 
# Actually This should be possible to get from Zephyr, but for
# some strange reason, they are corrupetd with some quotes.
set(MICROLIB_FLAGS -mcpu=cortex-m0plus -mthumb -mabi=aapcs -U_FORTIFY_SOURCE)

# Get optimization flag from Zephyr config
if(DEFINED CONFIG_COMPILER_OPT)
  # Extract just the optimization flag from CONFIG_COMPILER_OPT
  string(REGEX MATCH "-O[0-9s]" OPT_FLAG "${CONFIG_COMPILER_OPT}")
  if(OPT_FLAG)
    list(APPEND MICROLIB_FLAGS ${OPT_FLAG})
  endif()
else()
  # Default optimization if not specified
  list(APPEND MICROLIB_FLAGS -Os)
endif()

# Apply the flags to microlib
target_compile_options(microlib PRIVATE ${MICROLIB_FLAGS})
#target_compile_options(microlib PUBLIC ${options} -U_FORTIFY_SOURCE) 
#target_compile_definitions(microlib PUBLIC ${definitions})

target_include_directories(app PUBLIC "${MICROLIB_INCLUDE_DIR}" "${BSP_INCLUDE}" "${BOARD_INCLUDE}" "${PROJECT_BINARY_DIR}" src)
target_include_directories(microlib PUBLIC include)
target_link_libraries(app PUBLIC microlib)

# Sources for the smart sensor
set(SMART_SENSOR_SOURCES
        src/smart_sensors/smart_sensor_protocol.c
        src/smart_sensors/smart_sensor_innovex.c
        src/smart_sensors/smart_sensor_communication.c
        src/smart_sensors/smart_sensor_driver.c
        src/smart_sensors/smart_sensor_operations.c
        src/smart_sensors/smart_sensor_yosemitech.c
        src/smart_sensors/smart_sensor_ponsel.c
        src/smart_sensors/smart_sensor_ysi.c
        src/smart_sensors/smart_sensor_gps_pa1010d.c
        src/smart_sensors/smart_sensor_vaisala.c
        src/smart_sensors/smart_sensor_lufft.c
        src/smart_sensors/smart_sensor_huizhong.c
        src/smart_sensors/smart_sensor_signature_flow.c
        src/smart_sensors/smart_sensor_jiangsu_flow.c
        src/smart_sensors/smart_sensor_anb.c
        src/smart_sensors/smart_sensor_tds100.c
        src/smart_sensors/smart_sensor_chemins.c
        src/smart_sensors/smart_sensor_seabird.c
        src/smart_sensors/smart_sensor_signature_nortek.c
        src/smart_sensors/parsing_signature_nortek.c
        src/smart_sensors/smart_sensor_xm126.c
        src/smart_sensors/smart_sensor_aquadopp_nortek.c
        src/smart_sensors/smart_sensor_flowquest.c
        src/smart_sensors/parsing_flowquest.c
        src/smart_sensors/smart_sensor_wtvb01.c
)

# Add your source file to the "app" target. This must come after
# find_package(Zephyr) which defines the target.
target_sources(app PRIVATE
    src/main.c
    src/oxycontroller_ui.c
    src/sampling.c
    src/modbus.c
    src/measurement_operations.c
    src/shell_commands.c
    src/ui_valves.c
    src/oxygen_control.c
    src/comunication.c
    src/arch/zephyr/sensor_power_hw.c
    src/arch/zephyr/radio.c
    src/arch/zephyr/configuration.c
    src/arch/zephyr/solenoid_pca9538.c
    src/arch/zephyr/local_sensors.c
    src/arch/zephyr/watchdog.c
    src/arch/zephyr/measurement_storage.c
    src/soc/samd21/adc.c
    src/satellite_compression.c
    ${SMART_SENSOR_SOURCES}
    ${MICROLIB_ARCH_DIR}/serial.c
    ${MICROLIB_ARCH_DIR}/timing.c
    ${MICROLIB_ARCH_DIR}/lcdio.c
    ${MICROLIB_ARCH_DIR}/led-hw.c
    ${MICROLIB_ARCH_DIR}/power_hw.c
    ${MICROLIB_DIR}/src/multishell.c
    ${MICROLIB_DRIVERS_DIR}/display_driver_dog.c)
if (CONFIG_EXTERNAL_DATALOGGER)
    target_sources(app PRIVATE
        src/arch/zephyr/external_datalogger.c)
endif ()

