/*
 * Copyright (C) 2021, Xilinx Inc - All rights reserved
 * Xilinx Runtime (XRT) APIs
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You may
 * not use this file except in compliance with the License. A copy of the
 * License is located at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef xrtcore_pcie_noop_config_h_
#define xrtcore_pcie_noop_config_h_

//------------------Enable dynamic linking on noop-------------------------//

#ifdef _WIN32
# ifdef XRT_CORE_PCIE_NOOP_SOURCE
#  define XRT_CORE_PCIE_NOOP_EXPORT __declspec(dllexport)
# else
#  define XRT_CORE_PCIE_NOOP_EXPORT __declspec(dllimport)
# endif
#endif
#ifdef __linux__
# ifdef XRT_CORE_PCIE_NOOP_SOURCE
#  define XRT_CORE_PCIE_NOOP_EXPORT __attribute__ ((visibility("default")))
# else
#  define XRT_CORE_PCIE_NOOP_EXPORT
# endif
#endif

#ifndef XRT_CORE_PCIE_NOOP_EXPORT
# define XRT_CORE_PCIE_NOOP_EXPORT
#endif

#endif
