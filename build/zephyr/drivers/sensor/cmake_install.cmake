# Install script for directory: /home/seatech/my-workspace/zephyr/drivers/sensor

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/home/seatech/zephyr-sdk-0.17.4/xtensa-espressif_esp32_zephyr-elf/bin/xtensa-espressif_esp32_zephyr-elf-objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/adi/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/ams/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/aosong/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/asahi_kasei/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/bosch/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/broadcom/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/espressif/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/everlight/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/honeywell/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/infineon/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/ite/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/jedec/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/liteon/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/maxim/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/meas/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/melexis/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/memsic/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/microchip/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/nordic/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/nuvoton/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/nxp/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/omron/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/pixart/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/pni/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/realtek/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/renesas/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/rohm/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/seeed/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/sensirion/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/silabs/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/st/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/tdk/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/ti/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/vishay/cmake_install.cmake")
  include("/home/seatech/my-workspace/build/zephyr/drivers/sensor/wsen/cmake_install.cmake")

endif()

