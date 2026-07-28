// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/math/system/struct/GearboxStruct.hpp"

namespace {
constexpr size_t kMotorOff = 0;
constexpr size_t kNumMotorsOff =
    kMotorOff + wpi::util::GetStructSize<wpi::math::DCMotor>();
constexpr size_t kReductionOff = kNumMotorsOff + 4;
}  // namespace

using StructType = wpi::util::Struct<wpi::math::Gearbox>;

wpi::math::Gearbox StructType::Unpack(std::span<const uint8_t> data) {
  return wpi::math::Gearbox{
      wpi::util::UnpackStruct<wpi::math::DCMotor, kMotorOff>(data),
      wpi::util::UnpackStruct<int32_t, kNumMotorsOff>(data),
      wpi::util::UnpackStruct<double, kReductionOff>(data),
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const wpi::math::Gearbox& value) {
  wpi::util::PackStruct<kMotorOff>(data, value.motor);
  wpi::util::PackStruct<kNumMotorsOff>(data,
                                       static_cast<int32_t>(value.numMotors));
  wpi::util::PackStruct<kReductionOff>(data, value.reduction);
}
