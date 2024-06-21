# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/opt/nordic/ncs/v2.6.0/zephyr/samples/bluetooth/hci_ipc"
  "/Users/tomaitagaki/Documents/GitHub/millimobile-firmware/build/hci_ipc"
  "/Users/tomaitagaki/Documents/GitHub/millimobile-firmware/build/modules/nrf/samples/hci_ipc_subimage-prefix"
  "/Users/tomaitagaki/Documents/GitHub/millimobile-firmware/build/modules/nrf/samples/hci_ipc_subimage-prefix/tmp"
  "/Users/tomaitagaki/Documents/GitHub/millimobile-firmware/build/modules/nrf/samples/hci_ipc_subimage-prefix/src/hci_ipc_subimage-stamp"
  "/Users/tomaitagaki/Documents/GitHub/millimobile-firmware/build/modules/nrf/samples/hci_ipc_subimage-prefix/src"
  "/Users/tomaitagaki/Documents/GitHub/millimobile-firmware/build/modules/nrf/samples/hci_ipc_subimage-prefix/src/hci_ipc_subimage-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/tomaitagaki/Documents/GitHub/millimobile-firmware/build/modules/nrf/samples/hci_ipc_subimage-prefix/src/hci_ipc_subimage-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/tomaitagaki/Documents/GitHub/millimobile-firmware/build/modules/nrf/samples/hci_ipc_subimage-prefix/src/hci_ipc_subimage-stamp${cfgdir}") # cfgdir has leading slash
endif()
