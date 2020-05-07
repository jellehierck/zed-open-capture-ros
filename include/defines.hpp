#ifndef DEFINES_HPP
#define DEFINES_HPP

#include <stdint.h>
#include <string>
#include <string.h>
#include <cxxabi.h>
#include <iostream>
#include <vector>
#include <chrono>

#if defined _WIN32
#if defined(SL_SDK_COMPIL)
#define SL_SDK_EXPORT __declspec(dllexport)
#else
#define SL_SDK_EXPORT __declspec(dllimport)
#endif
#elif __GNUC__
#define SL_DRV_EXPORT __attribute__((visibility("default")))
#if defined(__arm__) || defined(__aarch64__)
#define _SL_JETSON_
#endif
#endif

//// SDK VERSION NUMBER
#define ZED_DRV_MAJOR_VERSION 0
#define ZED_DRV_MINOR_VERSION 1
#define ZED_DRV_PATCH_VERSION 0

#define ZED_DRV_VERSION_ATTRIBUTE private: uint32_t mDrvMajorVer = ZED_DRV_MAJOR_VERSION, mDrvMinorVer = ZED_DRV_MINOR_VERSION, mDrvPatchVer = ZED_DRV_PATCH_VERSION

// Debug output
#define INFO_OUT(msg) { int status_dem_0; std::cout << "[" << abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status_dem_0) << "] INFO: " << msg << std::endl; }
#define WARNING_OUT(msg) { int status_dem_0; std::cerr << "[" << abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status_dem_0) << "] WARNING: " << msg << std::endl; }
#define ERROR_OUT(msg) { int status_dem_0; std::cerr << "[" << abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status_dem_0) << "] ERROR: " << msg << std::endl; }

/*!
 * \brief Get the current system clock as steady clock, so with no jumps even if the system time changes
 * \return the current steady system clock in nanoseconds
 */
inline uint64_t getSteadyTimestamp() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}

/*!
 * \brief Get the current system clock as wall clock (it can have jumps if the system clock is updated by a sync service)
 * \return the current wall system clock in nanoseconds
 */
inline uint64_t getWallTimestamp() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();}

namespace sl_drv {

const uint16_t SL_USB_VENDOR = 0x2b03;           //!< Stereolabs Vendor ID

}

#endif //DEFINES_HPP
