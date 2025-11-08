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

// Provide fake header guard for cpplint
#undef ANYTIME_TRACING__TP_CALL_H_
#ifndef ANYTIME_TRACING__TP_CALL_H_
#define ANYTIME_TRACING__TP_CALL_H_

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER anytime

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "anytime_tracing/tp_call.h"

#if !defined(_ANYTIME_TRACING__TP_CALL_H_) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _ANYTIME_TRACING__TP_CALL_H_

#include <lttng/tracepoint.h>

#include <stdint.h>
#include <stdbool.h>

// Interference timer initialization tracepoint
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  interference_timer_init,
  TP_ARGS(
    const void *, node_handle_arg,
    const int, timer_period_ms_arg,
    const int, execution_time_ms_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, node_handle, node_handle_arg)
    ctf_integer(int, timer_period_ms, timer_period_ms_arg)
    ctf_integer(int, execution_time_ms, execution_time_ms_arg)
    ctf_string(version, anytime_tracing_VERSION)
  )
)

#endif  // _ANYTIME_TRACING__TP_CALL_H_

#include <lttng/tracepoint-event.h>

#endif  // ANYTIME_TRACING__TP_CALL_H_
