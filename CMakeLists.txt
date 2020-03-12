# Copyright (c) 2020, Fabian Gruber
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

project(delaunay)

set(DELAUNAY_BUILD_VIEWER "Build SDL based delaunay triangulization viewer" OFF)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

set(DELAUNAY_SOURCES src/delaunay/delaunay.cpp src/delaunay/delaunay.hpp)

function(delaunay_add_lib NAME_SUFFIX TYPE)
  set(LIB "delaunay-${NAME_SUFFIX}")

  add_library("${LIB}" "${TYPE}" ${DELAUNAY_SOURCES})
  target_include_directories("${LIB}" PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/delaunay>)
  target_compile_features("${LIB}" PUBLIC cxx_std_17)
  set_target_properties("${LIB}" PROPERTIES OUTPUT_NAME "delaunay")
  set_target_properties("${LIB}" PROPERTIES CXX_VISIBILITY_PRESET "hidden")
endfunction(delaunay_add_lib)

delaunay_add_lib(static STATIC)
delaunay_add_lib(dynamic SHARED)

if(DELAUNAY_BUILD_VIEWER)
  find_package(SDL2 REQUIRED)

  add_executable(delaunay-viewer util/viewer.cpp)
  target_link_libraries(delaunay-viewer PUBLIC delaunay-static ${SDL2_LIBRARIES})
endif(DELAUNAY_BUILD_VIEWER)
