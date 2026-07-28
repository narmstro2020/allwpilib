// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.system.struct;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Gearbox;

class GearboxStructTest {
  private static final Gearbox DATA =
      new Gearbox(new DCMotor(1.91, 19.1, 1.74, 1.74, 22.9), 3, 8.45);

  @Test
  void testRoundtrip() {
    ByteBuffer buffer = ByteBuffer.allocate(Gearbox.struct.getSize());
    buffer.order(ByteOrder.LITTLE_ENDIAN);
    Gearbox.struct.pack(buffer, DATA);
    buffer.rewind();

    Gearbox data = Gearbox.struct.unpack(buffer);
    assertEquals(DATA.numMotors, data.numMotors);
    assertEquals(DATA.reduction, data.reduction);
    assertEquals(DATA.motor.nominalVoltage, data.motor.nominalVoltage);
    assertEquals(DATA.motor.stallTorque, data.motor.stallTorque);
    assertEquals(DATA.motor.stallCurrent, data.motor.stallCurrent);
    assertEquals(DATA.motor.freeCurrent, data.motor.freeCurrent);
    assertEquals(DATA.motor.freeSpeed, data.motor.freeSpeed);
  }
}
