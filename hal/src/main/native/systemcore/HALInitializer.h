// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <atomic>

namespace hal::init {
extern std::atomic_bool HAL_IsInitialized;
extern void RunInitialize();
inline void CheckInit() {
  if (HAL_IsInitialized.load(std::memory_order_relaxed)) {
    return;
  }
  RunInitialize();
}

extern void InitializeCTREPCM();
extern void InitializeREVPH();
extern void InitializeAddressableLED();
extern void InitializeAnalogInput();
extern void InitializeAnalogInternal();
extern void InitializeCAN();
extern void InitializeCANAPI();
extern void InitializeConstants();
extern void InitializeCounter();
extern void InitializeDigitalInternal();
extern void InitializeDIO();
extern void InitializeDutyCycle();
extern void InitializeEncoder();
extern void InitializeFRCDriverStation();
extern void InitializeHAL();
extern void InitializeI2C();
extern void InitializeMain();
extern void InitializeNotifier();
extern void InitializeCTREPDP();
extern void InitializeREVPDH();
extern void InitializePorts();
extern void InitializePower();
extern void InitializePWM();
extern void InitializeSerialPort();
extern void InitializeSmartIo();
extern void InitializeThreads();
extern void InitializeUsageReporting();
}  // namespace hal::init
