// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Gearbox;
import org.wpilib.math.system.Models;

class FlywheelSimTest {
  @Test
  void testCurrentDrawScalesWithMotorCount() {
    // Current draw is the total across the gearbox's motors, so doubling the motor
    // count while holding the mechanism identical should double the stall current.
    double j = 0.00032;
    double reduction = 2.0;

    var oneMotor = new Gearbox(DCMotor.kNEO, 1, reduction);
    var twoMotors = new Gearbox(DCMotor.kNEO, 2, reduction);

    var oneMotorSim = new FlywheelSim(Models.flywheelFromPhysicalConstants(oneMotor, j), oneMotor);
    var twoMotorSim =
        new FlywheelSim(Models.flywheelFromPhysicalConstants(twoMotors, j), twoMotors);

    // At rest, current draw is the stall current at the applied voltage.
    oneMotorSim.setInput(12.0);
    twoMotorSim.setInput(12.0);

    assertEquals(DCMotor.kNEO.getCurrent(0.0, 12.0), oneMotorSim.getCurrentDraw(), 1e-9);
    assertEquals(2.0 * oneMotorSim.getCurrentDraw(), twoMotorSim.getCurrentDraw(), 1e-9);
  }

  @Test
  void testMomentOfInertiaRecoveredFromPlant() {
    // J is solved back out of the plant, so it should match what the plant was built with.
    double j = 0.00032;
    var gearbox = new Gearbox(DCMotor.kNEO, 2, 4.0);

    var sim = new FlywheelSim(Models.flywheelFromPhysicalConstants(gearbox, j), gearbox);

    assertEquals(j, sim.getJ(), 1e-12);
    assertEquals(gearbox, sim.getGearbox());
  }
}
