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

#include "anytime_tracing/anytime_tracetools.h"

#ifndef ANYTIME_TRACING_DISABLED

#ifdef ANYTIME_TRACING_LTTNG_ENABLED
# include "anytime_tracing/tp_call.h"
# define CONDITIONAL_TP(...) \
  tracepoint(TRACEPOINT_PROVIDER, __VA_ARGS__)
#else
# define CONDITIONAL_TP(...)
#endif

bool anytime_trace_compile_status()
{
#ifdef ANYTIME_TRACING_LTTNG_ENABLED
  return true;
#else
  return false;
#endif
}

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#else
# pragma warning(push)
# pragma warning(disable: 4100)
#endif

void ANYTIME_TRACEPOINT(
  interference_timer_init,
  const void * node_handle,
  const int timer_period_ms,
  const int execution_time_ms)
{
  CONDITIONAL_TP(
    interference_timer_init,
    node_handle,
    timer_period_ms,
    execution_time_ms);
}

#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif

#endif  // ANYTIME_TRACING_DISABLED
