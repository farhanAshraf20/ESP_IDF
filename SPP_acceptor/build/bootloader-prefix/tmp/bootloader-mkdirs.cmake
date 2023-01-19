# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v4.4.3/components/bootloader/subproject"
  "C:/F_ESP_workspace/SPP_acceptor/build/bootloader"
  "C:/F_ESP_workspace/SPP_acceptor/build/bootloader-prefix"
  "C:/F_ESP_workspace/SPP_acceptor/build/bootloader-prefix/tmp"
  "C:/F_ESP_workspace/SPP_acceptor/build/bootloader-prefix/src/bootloader-stamp"
  "C:/F_ESP_workspace/SPP_acceptor/build/bootloader-prefix/src"
  "C:/F_ESP_workspace/SPP_acceptor/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/F_ESP_workspace/SPP_acceptor/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
