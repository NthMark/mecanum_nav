// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef NAVIGATOR__VISIBILITY_CONTROL_H_
#define NAVIGATOR__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NAVIGATOR_EXPORT __attribute__ ((dllexport))
    #define NAVIGATOR_IMPORT __attribute__ ((dllimport))
  #else
    #define NAVIGATOR_EXPORT __declspec(dllexport)
    #define NAVIGATOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef NAVIGATOR_BUILDING_DLL
    #define NAVIGATOR_PUBLIC NAVIGATOR_EXPORT
  #else
    #define NAVIGATOR_PUBLIC NAVIGATOR_IMPORT
  #endif
  #define NAVIGATOR_PUBLIC_TYPE NAVIGATOR_PUBLIC
  #define NAVIGATOR_LOCAL
#else
  #define NAVIGATOR_EXPORT __attribute__ ((visibility("default")))
  #define NAVIGATOR_IMPORT
  #if __GNUC__ >= 4
    #define NAVIGATOR_PUBLIC __attribute__ ((visibility("default")))
    #define NAVIGATOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NAVIGATOR_PUBLIC
    #define NAVIGATOR_LOCAL
  #endif
  #define NAVIGATOR_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // NAVIGATOR__VISIBILITY_CONTROL_H_
