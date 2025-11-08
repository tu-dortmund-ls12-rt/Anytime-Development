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

#ifndef ANYTIME_TRACING__UTILS_HPP_
#define ANYTIME_TRACING__UTILS_HPP_

#include <string>

#include "anytime_tracing/visibility_control.hpp"

namespace anytime_tracing
{

/// Get symbol from function pointer.
/**
 * \param[in] funcptr the function pointer to resolve
 * \return the demangled symbol or an empty string if it cannot be resolved
 */
ANYTIME_TRACING_PUBLIC
std::string get_symbol(void * funcptr);

}  // namespace anytime_tracing

#endif  // ANYTIME_TRACING__UTILS_HPP_
