// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.system.struct;

import java.nio.ByteBuffer;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Gearbox;
import org.wpilib.util.struct.Struct;

public class GearboxStruct implements Struct<Gearbox> {
  @Override
  public Class<Gearbox> getTypeClass() {
    return Gearbox.class;
  }

  @Override
  public String getTypeName() {
    return "Gearbox";
  }

  @Override
  public int getSize() {
    return DCMotor.struct.getSize() + INT32_SIZE + DOUBLE_SIZE;
  }

  @Override
  public String getSchema() {
    return "DCMotor motor;int32 num_motors;double reduction";
  }

  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] {DCMotor.struct};
  }

  @Override
  public Gearbox unpack(ByteBuffer bb) {
    DCMotor motor = DCMotor.struct.unpack(bb);
    int numMotors = bb.getInt();
    double reduction = bb.getDouble();
    return new Gearbox(motor, numMotors, reduction);
  }

  @Override
  public void pack(ByteBuffer bb, Gearbox value) {
    DCMotor.struct.pack(bb, value.motor);
    bb.putInt(value.numMotors);
    bb.putDouble(value.reduction);
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
