// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.hardware.motor.PWMVictorSPX;
import org.wpilib.hardware.rotation.Encoder;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Gearbox;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.Models;
import org.wpilib.system.RobotController;

class GearboxSimTest {
  @Test
  void testVoltageSteadyState() {
    RoboRioSim.resetData();

    Gearbox gearbox = new Gearbox(DCMotor.kNEO, 1, 1);
    LinearSystem<N2, N1, N2> plant = Models.singleJointedArmFromPhysicalConstants(gearbox, 0.0005);
    GearboxSim sim = new GearboxSim(plant, gearbox);

    try (var motor = new PWMVictorSPX(0);
        var encoder = new Encoder(0, 1)) {
      var encoderSim = new EncoderSim(encoder);
      encoderSim.resetData();

      for (int i = 0; i < 100; i++) {
        motor.setVoltage(12);

        // ------ SimulationPeriodic() happens after user code -------
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDraw()));
        sim.setInputVoltage(motor.getThrottle() * RobotController.getBatteryVoltage());
        sim.update(0.020);
        encoderSim.setRate(sim.getAngularVelocity());
      }

      assertEquals(gearbox.motor.Kv * 12, encoder.getRate(), 0.1);

      for (int i = 0; i < 100; i++) {
        motor.setVoltage(0);

        // ------ SimulationPeriodic() happens after user code -------
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDraw()));
        sim.setInputVoltage(motor.getThrottle() * RobotController.getBatteryVoltage());
        sim.update(0.020);
        encoderSim.setRate(sim.getAngularVelocity());
      }

      assertEquals(0, encoder.getRate(), 0.1);
    }
  }

  @Test
  void testPositionFeedbackControl() {
    RoboRioSim.resetData();

    Gearbox gearbox = new Gearbox(DCMotor.kNEO, 1, 1);
    LinearSystem<N2, N1, N2> plant = Models.singleJointedArmFromPhysicalConstants(gearbox, 0.0005);
    GearboxSim sim = new GearboxSim(plant, gearbox);

    try (var motor = new PWMVictorSPX(0);
        var encoder = new Encoder(0, 1);
        var controller = new PIDController(0.04, 0.0, 0.001)) {
      var encoderSim = new EncoderSim(encoder);
      encoderSim.resetData();

      for (int i = 0; i < 140; i++) {
        motor.setThrottle(controller.calculate(encoder.getDistance(), 750));

        // ------ SimulationPeriodic() happens after user code -------
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDraw()));
        sim.setInputVoltage(motor.getThrottle() * RobotController.getBatteryVoltage());
        sim.update(0.020);
        encoderSim.setDistance(sim.getAngularPosition());
        encoderSim.setRate(sim.getAngularVelocity());
      }

      assertEquals(750, encoder.getDistance(), 1.0);
      assertEquals(0, encoder.getRate(), 0.1);
    }
  }

  @Test
  void testCurrentDrawScalesWithMotorCount() {
    // Current draw is the total across the gearbox's motors, so doubling the motor
    // count while holding the mechanism identical should double the stall current.
    double j = 0.0005;
    double reduction = 3.0;

    var oneMotor = new Gearbox(DCMotor.kNEO, 1, reduction);
    var twoMotors = new Gearbox(DCMotor.kNEO, 2, reduction);

    var oneMotorSim =
        new GearboxSim(Models.singleJointedArmFromPhysicalConstants(oneMotor, j), oneMotor);
    var twoMotorSim =
        new GearboxSim(Models.singleJointedArmFromPhysicalConstants(twoMotors, j), twoMotors);

    // At rest, current draw is the stall current at the applied voltage.
    oneMotorSim.setInput(12.0);
    twoMotorSim.setInput(12.0);

    assertEquals(DCMotor.kNEO.getCurrent(0.0, 12.0), oneMotorSim.getCurrentDraw(), 1e-9);
    assertEquals(2.0 * oneMotorSim.getCurrentDraw(), twoMotorSim.getCurrentDraw(), 1e-9);
  }
}
