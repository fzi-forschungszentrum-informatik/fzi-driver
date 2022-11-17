cmake_minimum_required(VERSION 3.5)
project(FZIDriver)

set(TARGET_SIMULATOR AUTO CACHE STRING "Set the simulator for which to compile.")
set(USE_IDM_MANEUVERS ON CACHE BOOL "Set to true to use IDM based maneuvers.")

include(ExternalProject)

# Set a default build type if none was specified
set(default_build_type "Release")
# configure build version
if (NOT CMAKE_BUILD_TYPE)
message(STATUS "Setting FZI Driver build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
               "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# FMU output directory
set(FMU_OUTDIR ${CMAKE_CURRENT_BINARY_DIR} CACHE STRING "Fmu output directory")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(LINK_WITH_SHARED_OSI ON CACHE BOOL "Link FMU with shared OSI library instead of statically linking")

# Logging
set(PUBLIC_LOGGING OFF CACHE BOOL "Enable logging via FMI logger")
set(PRIVATE_LOGGING OFF CACHE BOOL "Enable private logging to file")
if(WIN32)
	set(PRIVATE_LOG_PATH "C:/TEMP/${PROJECT_NAME}.log" CACHE FILEPATH "Path to write private log file to")
	set(PRIVATE_LOG_PATH "/tmp/${PROJECT_NAME}.log" CACHE FILEPATH "Path to write private log file to")
endif()
set(VERBOSE_FMI_LOGGING OFF CACHE BOOL "Enable detailed FMI function logging")
set(DEBUG_BREAKS OFF CACHE BOOL "Enable debugger traps for debug builds of FMU")

string(TIMESTAMP FMUTIMESTAMP UTC)
string(MD5 FMUGUID modelDescription.in.xml)

if (TARGET_SIMULATOR STREQUAL "OPEN_PASS")
	configure_file(modelDescription_OpenPass.xml modelDescription.xml @ONLY)
	set(MODEL_DESCRIPTION_FILE "modelDescription_OpenPass.xml")
else()
	configure_file(modelDescription.xml modelDescription.xml @ONLY)
	set(MODEL_DESCRIPTION_FILE "modelDescription.xml")
endif()

if(WIN32)
	set(PROTOBUF_CMAKE_GENERATOR "MSYS Makefiles")
	set(PROTOC_EXE "protoc.exe")
	set(OSI_CMAKE_GENERATOR "MSYS Makefiles")
elseif(UNIX)
	set(PROTOBUF_CMAKE_GENERATOR "Unix Makefiles")
	set(PROTOC_EXE "protoc")
	set(OSI_CMAKE_GENERATOR "Unix Makefiles")
endif()

# Download protobuf and install with position independent code 
ExternalProject_Add (protobuf_download
	PREFIX            	protobuf_download
	GIT_REPOSITORY    	https://github.com/protocolbuffers/protobuf.git
	GIT_TAG           	v3.11.4
	UPDATE_COMMAND    	""
	INSTALL_COMMAND   	make install
	BUILD_COMMAND     	make -j3
	BUILD_ALWAYS      	OFF
	INSTALL_DIR       	${CMAKE_CURRENT_BINARY_DIR}/ext/protobuf_download
	SOURCE_SUBDIR	  	cmake
	CONFIGURE_COMMAND 	${CMAKE_COMMAND} -G ${PROTOBUF_CMAKE_GENERATOR}
							-DCMAKE_BUILD_TYPE=Release 
							-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> 
							-DCMAKE_CXX_FLAGS="-fPIC" 
							-Dprotobuf_MSVC_STATIC_RUNTIME=OFF 
							-Dprotobuf_BUILD_TESTS=OFF 
							../protobuf_download/cmake)

# Download Osi and compile such that it links against the pic-build protobuf
ExternalProject_Add (osi_submodule
	DEPENDS            protobuf_download
	PREFIX             osi_download
	GIT_REPOSITORY     https://github.com/OpenSimulationInterface/open-simulation-interface.git
	GIT_TAG            v3.4.0
	SOURCE_DIR 	   	   ${CMAKE_SOURCE_DIR}/lib/open-simulation-interface
	UPDATE_COMMAND     ""
	INSTALL_COMMAND    make install
	BUILD_COMMAND      make -j3
	BUILD_ALWAYS       OFF
	INSTALL_DIR        ${CMAKE_CURRENT_BINARY_DIR}/ext/osi_submodule
	SOURCE_SUBDIR       cmake
	CONFIGURE_COMMAND 	${CMAKE_COMMAND} -G ${OSI_CMAKE_GENERATOR}
							-DCMAKE_BUILD_TYPE=Release 
							-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> 
							-DPROTOBUF_INCLUDE_DIR=${CMAKE_CURRENT_BINARY_DIR}/ext/protobuf_download/include 
							-DPROTOBUF_PROTOC_EXECUTABLE=${CMAKE_CURRENT_BINARY_DIR}/ext/protobuf_download/bin/${PROTOC_EXE} 
							-DPROTOBUF_LIBRARY=${CMAKE_CURRENT_BINARY_DIR}/ext/protobuf_download/lib/libprotobuf.a
							${CMAKE_SOURCE_DIR}/lib/open-simulation-interface)

# Get Osi installation directory
ExternalProject_Get_Property(osi_submodule INSTALL_DIR)
set(OSI ${INSTALL_DIR})
message("Osi installation directory :" ${OSI})

# Get Protobuf installation directory
ExternalProject_Get_Property(protobuf_download INSTALL_DIR)
set(PROTOBUF ${INSTALL_DIR})
message("Protobuf installation directory " ${PROTOBUF})

if (TARGET_SIMULATOR STREQUAL "OPEN_PASS")
	message("Bildung model for OpenPass.")
	add_compile_definitions(TARGET_OPEN_PASS)
else()
	message("Building default model.")
endif()

if (USE_IDM_MANEUVERS) 
	message("Using IDM based maneuvers.")
	add_compile_definitions(USE_IDM_MANEUVERS)
endif()

# Includes
set(FZIDRIVER_SRC ./TrafficAgent)
SET(PROTOBUF_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/ext/protobuf_download/include)
SET(OSI_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/ext/osi_submodule/include)

include_directories(
	${FZIDRIVER_SRC} 
	include/fmi
	${OSI_INCLUDE_DIR}
	${PROTOBUF_INCLUDE_DIR})

# Define target library
add_library(${PROJECT_NAME} SHARED 
	./TrafficAgent/OSMPTrafficAgent.cpp 
	./TrafficAgent/TrafficAgent.cpp 
	./TrafficAgent/TrafficParticipantBase.cpp
	./TrafficAgent/util/Interpolation.cpp
	./TrafficAgent/util/LinearInterpolation.cpp
	./TrafficAgent/util/LaneNetwork.cpp
	./TrafficAgent/util/Path.cpp)

# Build fmu after protobuf and osi has been downloaded and installed
add_dependencies(${PROJECT_NAME} protobuf_download osi_submodule)

# Compile defitions
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")
target_compile_definitions(${PROJECT_NAME} PRIVATE "FMU_SHARED_OBJECT")
target_compile_definitions(${PROJECT_NAME} PRIVATE "FMU_GUID=\"${FMUGUID}\"")

# Define libraries to link against
add_library(static_protobuf_lib STATIC IMPORTED)
add_dependencies(static_protobuf_lib protobuf_download)
set_target_properties(static_protobuf_lib PROPERTIES 
    IMPORTED_LOCATION "${PROTOBUF}/lib/libprotobuf.a")

add_library(static_osi_lib STATIC IMPORTED)
add_dependencies(static_osi_lib osi_submodule)
set_target_properties(static_osi_lib PROPERTIES 
	IMPORTED_LOCATION "${OSI}/lib/osi3/libopen_simulation_interface_pic.a")

if(WIN32)
	target_link_libraries(${PROJECT_NAME}
		PUBLIC static_osi_lib
		PUBLIC static_protobuf_lib)
elseif(UNIX)
	target_link_libraries(${PROJECT_NAME}
		PUBLIC static_osi_lib
		PUBLIC static_protobuf_lib)
endif()


if(PRIVATE_LOGGING)
	file(TO_NATIVE_PATH ${PRIVATE_LOG_PATH} PRIVATE_LOG_PATH_NATIVE)
	string(REPLACE "\\" "\\\\" PRIVATE_LOG_PATH_ESCAPED ${PRIVATE_LOG_PATH_NATIVE})
	target_compile_definitions(${PROJECT_NAME} PRIVATE
		"PRIVATE_LOG_PATH=\"${PRIVATE_LOG_PATH_ESCAPED}\"")
endif()
target_compile_definitions(${PROJECT_NAME} PRIVATE
	$<$<BOOL:${PUBLIC_LOGGING}>:PUBLIC_LOGGING>
	$<$<BOOL:${VERBOSE_FMI_LOGGING}>:VERBOSE_FMI_LOGGING>
	$<$<BOOL:${DEBUG_BREAKS}>:DEBUG_BREAKS>)

if(WIN32)
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(FMI_BINARIES_PLATFORM "win64")
	else()
		set(FMI_BINARIES_PLATFORM "win32")
	endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(FMI_BINARIES_PLATFORM "linux64")
	else()
		set(FMI_BINARIES_PLATFORM "linux32")
	endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(FMI_BINARIES_PLATFORM "darwin64")
	else()
		set(FMI_BINARIES_PLATFORM "darwin32")
	endif()
endif()

get_filename_component(Protobuf_REAL_LIBRARY_FILENAME "${Protobuf_LIBRARY}" REALPATH)

add_custom_command(OUTPUT ${FMU_OUTDIR}/${PROJECT_NAME}.fmu
	DEPENDS ${PROJECT_NAME}
			modelDescription.xml
	COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
	COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
	COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/${MODEL_DESCRIPTION_FILE}" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/modelDescription.xml"
	COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
	COMMAND ${CMAKE_COMMAND} -E chdir "${CMAKE_CURRENT_BINARY_DIR}/buildfmu" ${CMAKE_COMMAND} 
							 -E tar "cfv" "${FMU_OUTDIR}/${PROJECT_NAME}.fmu" 
							 --format=zip "modelDescription.xml" 
						 	 "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}")

add_custom_target(${PROJECT_NAME}_fmu
	ALL
	DEPENDS ${FMU_OUTDIR}/${PROJECT_NAME}.fmu)
