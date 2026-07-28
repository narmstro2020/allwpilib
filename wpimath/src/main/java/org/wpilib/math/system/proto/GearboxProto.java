// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.system.proto;

import org.wpilib.math.proto.ProtobufGearbox;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Gearbox;
import org.wpilib.util.protobuf.Protobuf;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class GearboxProto implements Protobuf<Gearbox, ProtobufGearbox> {
  @Override
  public Class<Gearbox> getTypeClass() {
    return Gearbox.class;
  }

  @Override
  public Descriptor getDescriptor() {
    return ProtobufGearbox.getDescriptor();
  }

  @Override
  public ProtobufGearbox createMessage() {
    return ProtobufGearbox.newInstance();
  }

  @Override
  public Gearbox unpack(ProtobufGearbox msg) {
    return new Gearbox(
        DCMotor.proto.unpack(msg.getMotor()), msg.getNumMotors(), msg.getReduction());
  }

  @Override
  public void pack(ProtobufGearbox msg, Gearbox value) {
    DCMotor.proto.pack(msg.getMutableMotor(), value.motor);
    msg.setNumMotors(value.numMotors);
    msg.setReduction(value.reduction);
  }
}
