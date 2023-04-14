cmake_minimum_required(VERSION 3.0.0)
project(MiSTer LANGUAGES CXX C)

## specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS_DEBUG}")
if (ENABLE_OPTIM)
	set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
endif()

exec_program("date" ARGS "+\"%y%m%d\"" OUTPUT_VARIABLE VDATE)
set(DFLAGS "-D_7ZIP_ST -DPACKAGE_VERSION=\"1.3.3\" -DFLAC_API_EXPORTS -DFLAC__HAS_OGG=0 -DHAVE_LROUND -DHAVE_STDINT_H -DHAVE_STDLIB_H -DHAVE_SYS_PARAM_H -DENABLE_64_BIT_WORDS=0 -D_FILE_OFFSET_BITS=64 -D_LARGEFILE64_SOURCE -DVDATE=\"${VDATE}\"")
set(CMAKE_CXX_FLAGS "${DFLAGS} ${CMAKE_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${DFLAGS} ${CMAKE_C_FLAGS}")

set(CMAKE_EXE_LINKER_FLAGS "-static-libstdc++ -static ${CMAKE_EXE_LINKER_FLAGS}")
set(BUILD_SHARED_LIBS OFF)

include(GNUInstallDirs)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
            "Choose the type of build, options are: None Debug Release."
            FORCE)
endif(NOT CMAKE_BUILD_TYPE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(BLUETOOTH REQUIRED bluez)
include_directories(${BLUETOOTH_INCLUDE_DIRS})

pkg_check_modules(FLAC REQUIRED flac)
pkg_check_modules(GPIOD REQUIRED libgpiod)
pkg_check_modules(IMLIB2 REQUIRED imlib2)
#pkg_check_modules( REQUIRED )

file(GLOB SOURCES "*.cpp" "lib/miniz/*.c" "lib/libchdr/*.c" "lib/lzma/*.c")
file(GLOB HEADERS "*.h"   "lib/miniz/*.h" "lib/libchdr/*.h")

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

include_directories(
    .
    /usr/include
    lib/miniz
    lib/md5
    lib/lzma
    lib/libchdr/include
    lib/serial_server/library
    ${BLUETOOTH_INCLUDE_DIRS}
    ${FLAC_INCLUDE_DIRS}
    ${GPIOD_INCLUDE_DIRS}
    ${IMLIB2_INCLUDE_DIRS}
)

#target_link_libraries()
