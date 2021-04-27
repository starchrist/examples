// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MINIMAL_ACTION_SERVER_AUTOWARE__VISIBILITY_CONTROL_HPP_
#define MINIMAL_ACTION_SERVER_AUTOWARE__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(MINIMAL_ACTION_SERVER_AUTOWARE_BUILDING_DLL) || defined(MINIMAL_ACTION_SERVER_AUTOWARE_EXPORTS)
    #define MINIMAL_ACTION_SERVER_AUTOWARE_PUBLIC __declspec(dllexport)
    #define MINIMAL_ACTION_SERVER_AUTOWARE_LOCAL
  #else  // defined(MINIMAL_ACTION_SERVER_AUTOWARE_BUILDING_DLL) || defined(MINIMAL_ACTION_SERVER_AUTOWARE_EXPORTS)
    #define MINIMAL_ACTION_SERVER_AUTOWARE_PUBLIC __declspec(dllimport)
    #define MINIMAL_ACTION_SERVER_AUTOWARE_LOCAL
  #endif  // defined(MINIMAL_ACTION_SERVER_AUTOWARE_BUILDING_DLL) || defined(MINIMAL_ACTION_SERVER_AUTOWARE_EXPORTS)
#elif defined(__linux__)
  #define MINIMAL_ACTION_SERVER_AUTOWARE_PUBLIC __attribute__((visibility("default")))
  #define MINIMAL_ACTION_SERVER_AUTOWARE_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define MINIMAL_ACTION_SERVER_AUTOWARE_PUBLIC __attribute__((visibility("default")))
  #define MINIMAL_ACTION_SERVER_AUTOWARE_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // MINIMAL_ACTION_SERVER_AUTOWARE__VISIBILITY_CONTROL_HPP_
