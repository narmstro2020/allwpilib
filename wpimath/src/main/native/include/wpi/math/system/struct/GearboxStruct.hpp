// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "wpi/math/system/Gearbox.hpp"
#include "wpi/util/SymbolExports.hpp"
#include "wpi/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT wpi::util::Struct<wpi::math::Gearbox> final {
  static constexpr std::string_view GetTypeName() { return "Gearbox"; }
  static constexpr size_t GetSize() {
    return wpi::util::GetStructSize<wpi::math::DCMotor>() + 4 + 8;
  }
  static constexpr std::string_view GetSchema() {
    return "DCMotor motor;int32 num_motors;double reduction";
  }

  static wpi::math::Gearbox Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data, const wpi::math::Gearbox& value);
  static void ForEachNested(
      std::invocable<std::string_view, std::string_view> auto fn) {
    wpi::util::ForEachStructSchema<wpi::math::DCMotor>(fn);
  }
};

static_assert(wpi::util::StructSerializable<wpi::math::Gearbox>);
static_assert(wpi::util::HasNestedStruct<wpi::math::Gearbox>);
