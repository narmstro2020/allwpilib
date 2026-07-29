// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.hardware.motor.PWMVictorSPX;
import org.wpilib.hardware.rotation.Encoder;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Gearbox;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.system.RobotController;

class ElevatorSimTest {
  @Test
  void testStateSpaceSimWithElevator() {
    RoboRioSim.resetData();

    @SuppressWarnings("resource")
    var controller = new PIDController(10, 0, 0);

    var sim =
        new ElevatorSim(
            new Gearbox(DCMotor.kVex775Pro, 4, 14.67),
            8,
            0.75 * 25.4 / 1000.0,
            0.0,
            3.0,
            true,
            0.0,
            0.01,
            0.0);

    try (var motor = new PWMVictorSPX(0);
        var encoder = new Encoder(0, 1)) {
      var encoderSim = new EncoderSim(encoder);

      for (int i = 0; i < 100; i++) {
        controller.setSetpoint(2.0);

        double nextVoltage = controller.calculate(encoderSim.getDistance());

        double currentBatteryVoltage = RobotController.getBatteryVoltage();
        motor.setThrottle(nextVoltage / currentBatteryVoltage);

        // ------ SimulationPeriodic() happens after user code -------

        var u = VecBuilder.fill(motor.getThrottle() * currentBatteryVoltage);
        sim.setInput(u);
        sim.update(0.020);
        var y = sim.getOutput();
        encoderSim.setDistance(y.get(0, 0));
      }

      assertEquals(controller.getSetpoint(), sim.getPosition(), 0.2);
    }
  }

  @Test
  void testInitialState() {
    double startingHeightMeters = 0.5;
    var sim =
        new ElevatorSim(
            new Gearbox(DCMotor.kKrakenX60, 2, 20),
            8.0,
            0.1,
            0.0,
            1.0,
            true,
            startingHeightMeters,
            0.01,
            0.0);

    assertEquals(startingHeightMeters, sim.getPosition());
    assertEquals(0, sim.getVelocity());
  }

  @Test
  void testMinMax() {
    var sim =
        new ElevatorSim(
            new Gearbox(DCMotor.kVex775Pro, 4, 14.67),
            8.0,
            0.75 * 25.4 / 1000.0,
            0.0,
            1.0,
            true,
            0.0);

    for (int i = 0; i < 100; i++) {
      sim.setInput(VecBuilder.fill(0));
      sim.update(0.020);
      var height = sim.getPosition();
      assertTrue(height >= 0.0);
    }

    for (int i = 0; i < 100; i++) {
      sim.setInput(VecBuilder.fill(12.0));
      sim.update(0.020);
      var height = sim.getPosition();
      assertTrue(height <= 1.0);
    }
  }

  @Test
  void testStability() {
    var sim =
        new ElevatorSim(
            new Gearbox(DCMotor.kVex775Pro, 4, 100),
            4,
            Units.inchesToMeters(0.5),
            0,
            10,
            false,
            0.0);

    sim.setState(VecBuilder.fill(0, 0));
    sim.setInput(12);
    for (int i = 0; i < 50; ++i) {
      sim.update(0.02);
    }

    var system =
        Models.elevatorFromPhysicalConstants(
            new Gearbox(DCMotor.kVex775Pro, 4, 100), 4, Units.inchesToMeters(0.5));
    assertEquals(
        system.calculateX(VecBuilder.fill(0, 0), VecBuilder.fill(12), 0.02 * 50.0).get(0, 0),
        sim.getPosition(),
        0.01);
  }

  @Test
  void testCurrentDraw() {
    var gearbox = new Gearbox(DCMotor.kKrakenX60, 2, 20);
    var sim = new ElevatorSim(gearbox, 8.0, 0.1, 0.0, 1.0, true, 0.0, 0.01, 0.0);

    assertEquals(0.0, sim.getCurrentDraw());
    // Drive the gearbox with the voltage its motors need to produce 60 A of
    // total draw at zero speed.
    var motor = gearbox.motor;
    sim.setInputVoltage(motor.getVoltage(motor.getTorque(60.0 / gearbox.numMotors), 0.0));
    sim.update(0.100);
    // Current draw should start at 60 A and decrease as the back-EMF catches up
    assertTrue(0.0 < sim.getCurrentDraw() && sim.getCurrentDraw() < 60.0);
  }

  @Test
  void testCurrentDrawAtSpeed() {
    // At a standstill the reduction cancels out of the back-EMF term, so this uses a
    // non-zero velocity and a reduction other than one to pin down the conversion from
    // carriage velocity to motor velocity.
    double reduction = 10.0;
    double drumRadius = 0.05;
    double velocity = 1.0;
    double voltage = 12.0;

    var gearbox = new Gearbox(DCMotor.kNEO, 2, reduction);
    var sim = new ElevatorSim(gearbox, 8.0, drumRadius, 0.0, 10.0, false, 0.0);

    sim.setState(1.0, velocity);
    sim.setInput(voltage);

    // v = r·ω at the drum, and the motors turn `reduction` times faster than the drum.
    double motorVelocity = velocity / drumRadius * reduction;
    double expected = gearbox.numMotors * DCMotor.kNEO.getCurrent(motorVelocity, voltage);

    assertEquals(expected, sim.getCurrentDraw(), 1e-6);
  }
}
