// Copyright (c) 2023, SENAI Cimatec
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PRESSURE_PKG__VISIBILITY_CONTROL_H_
#define PRESSURE_PKG__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PRESSURE_PKG_EXPORT __attribute__ ((dllexport))
    #define PRESSURE_PKG_IMPORT __attribute__ ((dllimport))
  #else
    #define PRESSURE_PKG_EXPORT __declspec(dllexport)
    #define PRESSURE_PKG_IMPORT __declspec(dllimport)
  #endif
  #ifdef PRESSURE_PKG_BUILDING_LIBRARY
    #define PRESSURE_PKG_PUBLIC PRESSURE_PKG_EXPORT
  #else
    #define PRESSURE_PKG_PUBLIC PRESSURE_PKG_IMPORT
  #endif
  #define PRESSURE_PKG_PUBLIC_TYPE PRESSURE_PKG_PUBLIC
  #define PRESSURE_PKG_LOCAL
#else
  #define PRESSURE_PKG_EXPORT __attribute__ ((visibility("default")))
  #define PRESSURE_PKG_IMPORT
  #if __GNUC__ >= 4
    #define PRESSURE_PKG_PUBLIC __attribute__ ((visibility("default")))
    #define PRESSURE_PKG_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PRESSURE_PKG_PUBLIC
    #define PRESSURE_PKG_LOCAL
  #endif
  #define PRESSURE_PKG_PUBLIC_TYPE
#endif

#endif  // PRESSURE_PKG__VISIBILITY_CONTROL_H_
