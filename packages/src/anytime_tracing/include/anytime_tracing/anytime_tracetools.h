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

/** \mainpage anytime_tracing: custom tracing instrumentation for anytime system
 *
 * `anytime_tracing` provides utilities to instrument anytime system components.
 * It provides tracepoints for performance analysis and debugging.
 */

#ifndef ANYTIME_TRACING__ANYTIME_TRACETOOLS_H_
#define ANYTIME_TRACING__ANYTIME_TRACETOOLS_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "anytime_tracing/config.h"
#include "anytime_tracing/visibility_control.hpp"

#ifndef ANYTIME_TRACING_DISABLED
/**
 * This allows us to select between two versions of each macro
 * to avoid the 'gnu-zero-variadic-macro-arguments' warning:
 *    1. Only one macro argument for tracepoints without any arguments.
 *    2. Up to 10 macro arguments for tracepoints with up to 9 arguments.
 */
#  define _ANYTIME_GET_MACRO(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, NAME, ...) NAME

#  define _ANYTIME_TRACEPOINT_NOARGS(event_name) \
  (anytime_trace_ ## event_name)()
#  define _ANYTIME_TRACEPOINT_ARGS(event_name, ...) \
  (anytime_trace_ ## event_name)(__VA_ARGS__)
#  define _ANYTIME_DECLARE_TRACEPOINT_NOARGS(event_name) \
  ANYTIME_TRACING_PUBLIC void anytime_trace_ ## event_name();
#  define _ANYTIME_DECLARE_TRACEPOINT_ARGS(event_name, ...) \
  ANYTIME_TRACING_PUBLIC void anytime_trace_ ## event_name(__VA_ARGS__);

#  define _ANYTIME_GET_MACRO_TRACEPOINT(...) \
  _ANYTIME_GET_MACRO( \
    __VA_ARGS__, \
    _ANYTIME_TRACEPOINT_ARGS, _ANYTIME_TRACEPOINT_ARGS, _ANYTIME_TRACEPOINT_ARGS, \
    _ANYTIME_TRACEPOINT_ARGS, _ANYTIME_TRACEPOINT_ARGS, _ANYTIME_TRACEPOINT_ARGS, \
    _ANYTIME_TRACEPOINT_ARGS, _ANYTIME_TRACEPOINT_ARGS, _ANYTIME_TRACEPOINT_ARGS, \
    _ANYTIME_TRACEPOINT_NOARGS, should_not_be_called_without_any_arguments)
#  define _ANYTIME_GET_MACRO_DECLARE_TRACEPOINT(...) \
  _ANYTIME_GET_MACRO( \
    __VA_ARGS__, \
    _ANYTIME_DECLARE_TRACEPOINT_ARGS, _ANYTIME_DECLARE_TRACEPOINT_ARGS, \
    _ANYTIME_DECLARE_TRACEPOINT_ARGS, _ANYTIME_DECLARE_TRACEPOINT_ARGS, \
    _ANYTIME_DECLARE_TRACEPOINT_ARGS, _ANYTIME_DECLARE_TRACEPOINT_ARGS, \
    _ANYTIME_DECLARE_TRACEPOINT_ARGS, _ANYTIME_DECLARE_TRACEPOINT_ARGS, \
    _ANYTIME_DECLARE_TRACEPOINT_ARGS, _ANYTIME_DECLARE_TRACEPOINT_NOARGS, \
    should_not_be_called_without_any_arguments)

/// Call a tracepoint.
/**
 * The first argument is mandatory and should be the tracepoint event name.
 * The other arguments should be the tracepoint arguments.
 * This is the preferred method over calling the actual function directly.
 *
 * This macro currently supports up to 9 tracepoint arguments after the event name.
 */
#  define ANYTIME_TRACEPOINT(...) \
  _ANYTIME_GET_MACRO_TRACEPOINT(__VA_ARGS__)(__VA_ARGS__)
#  define ANYTIME_DECLARE_TRACEPOINT(...) \
  _ANYTIME_GET_MACRO_DECLARE_TRACEPOINT(__VA_ARGS__)(__VA_ARGS__)
#else
#  define ANYTIME_TRACEPOINT(...) ((void) (0))
#  define ANYTIME_DECLARE_TRACEPOINT(...)
#endif  // ANYTIME_TRACING_DISABLED

#ifdef __cplusplus
extern "C"
{
#endif

/// Get tracing compilation status.
/**
 * \return `true` if tracing is enabled, `false` otherwise
 */
ANYTIME_TRACING_PUBLIC bool anytime_trace_compile_status();

/// `interference_timer_init`
/**
 * Interference timer node initialization.
 * Records when an interference timer node is created with its configuration parameters.
 *
 * \param[in] node_handle pointer to the node handle
 * \param[in] timer_period_ms timer period in milliseconds
 * \param[in] execution_time_ms execution time (busy-wait duration) in milliseconds
 */
ANYTIME_DECLARE_TRACEPOINT(
  interference_timer_init,
  const void * node_handle,
  const int timer_period_ms,
  const int execution_time_ms)

#ifdef __cplusplus
}
#endif

#endif  // ANYTIME_TRACING__ANYTIME_TRACETOOLS_H_
