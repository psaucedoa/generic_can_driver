/*
 * Copyright 2024 Construction Engineering Research Laboratory (CERL)
 * Engineer Reseach and Development Center (ERDC)
 * U.S. Army Corps of Engineers
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef GENERIC_CAN_DRIVER__VISIBILITY_CONTROL_HPP_
#define GENERIC_CAN_DRIVER__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define GENERIC_DRIVER_EXPORT __attribute__((dllexport))
#define GENERIC_DRIVER_IMPORT __attribute__((dllimport))
#else
#define GENERIC_DRIVER_EXPORT __declspec(dllexport)
#define GENERIC_DRIVER_IMPORT __declspec(dllimport)
#endif
#ifdef GENERIC_DRIVER_BUILDING_LIBRARY
#define GENERIC_DRIVER_PUBLIC GENERIC_DRIVER_EXPORT
#else
#define GENERIC_DRIVER_PUBLIC GENERIC_DRIVER_IMPORT
#endif  // GENERIC_CAN_DRIVER__VISIBILITY_CONTROL_HPP_
#define GENERIC_DRIVER_PUBLIC_TYPE GENERIC_DRIVER_PUBLIC
#define GENERIC_DRIVER_LOCAL
#else
#define GENERIC_DRIVER_EXPORT __attribute__((visibility("default")))
#define GENERIC_DRIVER_IMPORT
#if __GNUC__ >= 4
#define GENERIC_DRIVER_PUBLIC __attribute__((visibility("default")))
#define GENERIC_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else
#define GENERIC_DRIVER_PUBLIC
#define GENERIC_DRIVER_LOCAL
#endif
#define GENERIC_DRIVER_PUBLIC_TYPE
#endif

#endif  // GENERIC_CAN_DRIVER__VISIBILITY_CONTROL_HPP_
