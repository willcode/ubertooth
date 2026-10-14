#
# This file is part of Ubertooth.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#
# Based slightly upon the hackrf cmake setup.

include(${PROJECT_SOURCE_DIR}/../cmake/set_release.cmake)
add_definitions( -DRELEASE="${RELEASE_STRING}" )
message(STATUS "Setting version string ${RELEASE_STRING}")

# Targets
set(c_sources ${CMAKE_CURRENT_SOURCE_DIR}/ubertooth.c
              ${CMAKE_CURRENT_SOURCE_DIR}/ubertooth_callback.c
              ${CMAKE_CURRENT_SOURCE_DIR}/ubertooth_control.c
              ${CMAKE_CURRENT_SOURCE_DIR}/ubertooth_fifo.c
              ${CMAKE_CURRENT_SOURCE_DIR}/ubertooth_le_recover.c
			  CACHE INTERNAL "List of C sources")
set(c_headers ${CMAKE_CURRENT_SOURCE_DIR}/ubertooth.h
              ${CMAKE_CURRENT_SOURCE_DIR}/ubertooth_callback.h
              ${CMAKE_CURRENT_SOURCE_DIR}/ubertooth_control.h
              ${CMAKE_CURRENT_SOURCE_DIR}/ubertooth_fifo.h
			  ${CMAKE_CURRENT_SOURCE_DIR}/ubertooth_interface.h
              ${CMAKE_CURRENT_SOURCE_DIR}/ubertooth_le_recover.h
			  CACHE INTERNAL "List of C headers")

# Include and link to libbtbb and libusb-1.0
find_package(BTBB REQUIRED)
find_package(USB1 REQUIRED)

find_package(Threads REQUIRED)

include_directories(${LIBUSB_INCLUDE_DIR} ${LIBBTBB_INCLUDE_DIR})
LIST(APPEND LIBUBERTOOTH_LIBS ${LIBUSB_LIBRARIES} ${LIBBTBB_LIBRARIES} Threads::Threads)

# Optional HackRF Pro backend (BR channelizer gateware): needs libhackrf
find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
	pkg_check_modules(LIBHACKRF libhackrf)
endif()
if(LIBHACKRF_FOUND)
	message(STATUS "Building the HackRF Pro backend (libhackrf ${LIBHACKRF_VERSION})")
	add_definitions(-DHAVE_LIBHACKRF)
	include_directories(${LIBHACKRF_INCLUDE_DIRS})
	link_directories(${LIBHACKRF_LIBRARY_DIRS})
	LIST(APPEND LIBUBERTOOTH_LIBS ${LIBHACKRF_LIBRARIES})
	set(c_sources ${c_sources} ${CMAKE_CURRENT_SOURCE_DIR}/ubertooth_hackrf.c
		CACHE INTERNAL "List of C sources")
	# the gateware loaded into a selected HackRF Pro: the BR channelizer
	# image built by hackrf-ubertooth (fpga-build/packs/br_rx.pack), installed
	# when UBERTOOTH_HACKRF_PACK_FILE names it; UBERTOOTH_HACKRF_PACK in the
	# environment overrides the path at run time
	set(UBERTOOTH_HACKRF_PACK_FILE "" CACHE FILEPATH
		"BR channelizer gateware for the HackRF Pro backend (br_rx.pack) to install")
	set(UBERTOOTH_HACKRF_PACK_DIR ${CMAKE_INSTALL_PREFIX}/share/ubertooth)
	add_definitions(-DUBERTOOTH_HACKRF_PACK_DEFAULT="${UBERTOOTH_HACKRF_PACK_DIR}/br_rx.pack")
	if(UBERTOOTH_HACKRF_PACK_FILE)
		install(FILES ${UBERTOOTH_HACKRF_PACK_FILE} DESTINATION ${UBERTOOTH_HACKRF_PACK_DIR}
			RENAME br_rx.pack)
	else()
		message(STATUS "UBERTOOTH_HACKRF_PACK_FILE not set: the HackRF Pro gateware is not installed")
	endif()
else()
	message(STATUS "libhackrf not found: no HackRF Pro backend")
endif()

# Optional libbtbb features (newer libbtbb releases)
include(CheckSymbolExists)
include(CMakePushCheckState)
cmake_push_check_state(RESET)
set(CMAKE_REQUIRED_INCLUDES ${LIBBTBB_INCLUDE_DIR})
set(CMAKE_REQUIRED_LIBRARIES ${LIBBTBB_LIBRARIES})
check_symbol_exists(lell_allocate_and_decode_len "btbb.h" HAVE_LELL_ALLOCATE_AND_DECODE_LEN)
check_symbol_exists(lell_packet_set_encrypted "btbb.h" HAVE_LELL_PACKET_SET_ENCRYPTED)
check_symbol_exists(lell_packet_set_direction "btbb.h" HAVE_LELL_PACKET_SET_DIRECTION)
check_symbol_exists(btbb_find_ac_checked "btbb.h" HAVE_BTBB_FIND_AC_CHECKED)
check_symbol_exists(btbb_set_behind "btbb.h" HAVE_BTBB_SET_BEHIND)
cmake_pop_check_state()
if(HAVE_LELL_ALLOCATE_AND_DECODE_LEN)
	add_definitions(-DHAVE_LELL_ALLOCATE_AND_DECODE_LEN)
else()
	message(WARNING "libbtbb lacks lell_allocate_and_decode_len(): LE packets longer than 64 bytes will not be decoded")
endif()
if(HAVE_BTBB_FIND_AC_CHECKED)
	add_definitions(-DHAVE_BTBB_FIND_AC_CHECKED)
endif()
if(HAVE_BTBB_SET_BEHIND)
	add_definitions(-DHAVE_BTBB_SET_BEHIND)
endif()
if(HAVE_LELL_PACKET_SET_DIRECTION)
	add_definitions(-DHAVE_LELL_PACKET_SET_DIRECTION)
endif()
if(HAVE_LELL_PACKET_SET_ENCRYPTED)
	add_definitions(-DHAVE_LELL_PACKET_SET_ENCRYPTED)
endif()

if( ${BUILD_SHARED_LIB} )
	# Shared library
	message(STATUS "Building shared library")
	add_library(ubertooth SHARED ${c_sources})
	target_link_libraries(ubertooth ${LIBUBERTOOTH_LIBS})
	set_target_properties(ubertooth PROPERTIES VERSION ${MAJOR_VERSION}.${MINOR_VERSION} SOVERSION 1)
	set_target_properties(ubertooth PROPERTIES CLEAN_DIRECT_OUTPUT 1)
	install(TARGETS ubertooth
			LIBRARY DESTINATION lib${LIB_SUFFIX}
			COMPONENT sharedlibs)
endif( ${BUILD_SHARED_LIB} )

if( ${BUILD_STATIC_LIB} )
	# Static library
	message(STATUS "Building static library")
	add_library(ubertooth-static STATIC ${c_sources})
	target_link_libraries(ubertooth-static ${LIBUBERTOOTH_LIBS})
	set_target_properties(ubertooth-static PROPERTIES OUTPUT_NAME "ubertooth")
	set_target_properties(ubertooth-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
	install(TARGETS ubertooth-static
			DESTINATION lib${LIB_SUFFIX}
			COMPONENT staticlibs)
endif( ${BUILD_STATIC_LIB} )

install(FILES ${c_headers}
		DESTINATION include
		COMPONENT headers)
