// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/math/system/proto/GearboxProto.hpp"

#include <optional>

#include "wpi/util/protobuf/ProtobufCallbacks.hpp"
#include "wpimath/protobuf/system.npb.h"

std::optional<wpi::math::Gearbox>
wpi::util::Protobuf<wpi::math::Gearbox>::Unpack(InputStream& stream) {
  wpi::util::UnpackCallback<wpi::math::DCMotor> motor;
  wpi_proto_ProtobufGearbox msg{
      .motor = motor.Callback(),
      .num_motors = 0,
      .reduction = 0.0,
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  auto imotor = motor.Items();
  if (imotor.empty()) {
    return {};
  }

  return wpi::math::Gearbox{
      imotor[0],
      static_cast<int>(msg.num_motors),
      msg.reduction,
  };
}

bool wpi::util::Protobuf<wpi::math::Gearbox>::Pack(
    OutputStream& stream, const wpi::math::Gearbox& value) {
  wpi::util::PackCallback motor{&value.motor};
  wpi_proto_ProtobufGearbox msg{
      .motor = motor.Callback(),
      .num_motors = static_cast<uint32_t>(value.numMotors),
      .reduction = value.reduction,
  };
  return stream.Encode(msg);
}
