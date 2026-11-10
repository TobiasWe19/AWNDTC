cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(assistive_wristband_ndt_v4_experimental)

# Build different firmware for every PlayerN, keeping identical source.
set(MY_WRISTBAND_ID 3 CACHE STRING "Player ID flashed to this board")
set(NUM_WRISTBANDS 4 CACHE STRING "Even number of wristbands")
set(ENABLE_PERIODIC 1 CACHE STRING "1=legacy+periodic; 0=legacy only for NDT debugging")
target_compile_definitions(app PRIVATE
  ENABLE_PERIODIC=${ENABLE_PERIODIC}
  MY_WRISTBAND_ID=${MY_WRISTBAND_ID}
  NUM_WRISTBANDS=${NUM_WRISTBANDS}
)
target_sources(app PRIVATE src/main.c)
message(STATUS "WRISTBAND BUILD: Player${MY_WRISTBAND_ID} of ${NUM_WRISTBANDS}, ENABLE_PERIODIC=${ENABLE_PERIODIC}")