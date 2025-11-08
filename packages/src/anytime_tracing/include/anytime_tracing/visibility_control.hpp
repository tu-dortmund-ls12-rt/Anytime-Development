// Copyright 2025 Anytime System
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

#ifndef ANYTIME_TRACING__VISIBILITY_CONTROL_HPP_
#define ANYTIME_TRACING__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ANYTIME_TRACING_EXPORT __attribute__ ((dllexport))
    #define ANYTIME_TRACING_IMPORT __attribute__ ((dllimport))
  #else
    #define ANYTIME_TRACING_EXPORT __declspec(dllexport)
    #define ANYTIME_TRACING_IMPORT __declspec(dllimport)
  #endif
  #ifdef ANYTIME_TRACING_BUILDING_DLL
    #define ANYTIME_TRACING_PUBLIC ANYTIME_TRACING_EXPORT
  #else
    #define ANYTIME_TRACING_PUBLIC ANYTIME_TRACING_IMPORT
  #endif
  #define ANYTIME_TRACING_PUBLIC_TYPE ANYTIME_TRACING_PUBLIC
  #define ANYTIME_TRACING_LOCAL
#else
  #define ANYTIME_TRACING_EXPORT __attribute__ ((visibility("default")))
  #define ANYTIME_TRACING_IMPORT
  #if __GNUC__ >= 4
    #define ANYTIME_TRACING_PUBLIC __attribute__ ((visibility("default")))
    #define ANYTIME_TRACING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ANYTIME_TRACING_PUBLIC
    #define ANYTIME_TRACING_LOCAL
  #endif
  #define ANYTIME_TRACING_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ANYTIME_TRACING__VISIBILITY_CONTROL_HPP_
