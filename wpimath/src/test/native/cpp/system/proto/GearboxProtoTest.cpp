// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "wpi/math/system/Gearbox.hpp"
#include "wpi/util/SmallVector.hpp"

using namespace wpi::math;

using ProtoType = wpi::util::Protobuf<wpi::math::Gearbox>;

inline constexpr Gearbox kExpectedData =
    Gearbox{DCMotor{1.91_V, 19.1_Nm, 1.74_A, 2.29_A, 2.2_rad_per_s}, 3, 8.45};

TEST(GearboxProtoTest, Roundtrip) {
  wpi::util::ProtobufMessage<decltype(kExpectedData)> message;
  wpi::util::SmallVector<uint8_t, 64> buf;

  ASSERT_TRUE(message.Pack(buf, kExpectedData));
  auto unpacked_data = message.Unpack(buf);
  ASSERT_TRUE(unpacked_data.has_value());

  EXPECT_EQ(kExpectedData.numMotors, unpacked_data->numMotors);
  EXPECT_EQ(kExpectedData.reduction, unpacked_data->reduction);
  EXPECT_EQ(kExpectedData.motor.nominalVoltage.value(),
            unpacked_data->motor.nominalVoltage.value());
  EXPECT_EQ(kExpectedData.motor.stallTorque.value(),
            unpacked_data->motor.stallTorque.value());
  EXPECT_EQ(kExpectedData.motor.stallCurrent.value(),
            unpacked_data->motor.stallCurrent.value());
  EXPECT_EQ(kExpectedData.motor.freeCurrent.value(),
            unpacked_data->motor.freeCurrent.value());
  EXPECT_EQ(kExpectedData.motor.freeSpeed.value(),
            unpacked_data->motor.freeSpeed.value());
}
