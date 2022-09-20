// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.misc.util.CircleConverter;
import frc.robot.misc.util.GearingConverter;
import frc.robot.misc.util.sensors.SensorUnitConverter;

public class SwerveModuleDrive {
  private static final double MAX_VOLTAGE = 12;
  private static final CircleConverter WHEEL_CONVERTER = CircleConverter.fromDiameter(6);
  private static final GearingConverter GEARING_CONVERTER = GearingConverter.fromReduction(10);
  private static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0, 0);

  private final TalonFX motor;

  /** Creates a new SwerveModuleDrive. */
  public SwerveModuleDrive(TalonFX motor) {
    this.motor = motor;

    motor.config_kP(0, 0);
    motor.config_kI(0, 0);
    motor.config_kD(0, 0);
    motor.config_kF(0, 0);

    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));
  }

  public void setGoalVelocity(double velocity) {
    final var feetPerSecond = velocity;
    final var feetPerMinute = feetPerSecond * 60;
    final var rotationsPerMinute = WHEEL_CONVERTER.distanceToRotations(feetPerMinute);
    final var sensorUnitsPer100ms =
        SensorUnitConverter.talonFX.rotationsPerMinuteToSensorUnitsPer100ms(rotationsPerMinute);
    final var sensorUnitsPer100msBeforeGearing =
        GEARING_CONVERTER.afterToBeforeGearing(sensorUnitsPer100ms);

    motor.set(
        ControlMode.Velocity,
        sensorUnitsPer100msBeforeGearing,
        DemandType.ArbitraryFeedForward,
        FEEDFORWARD.kv / MAX_VOLTAGE);
  }

  public double getVelocity() {
    final var sensorUnitsPer100msBeforeGearing = motor.getSelectedSensorVelocity();
    final var sensorUnitsPer100ms =
        GEARING_CONVERTER.beforeToAfterGearing(sensorUnitsPer100msBeforeGearing);
    final var rotationsPerMinute =
        SensorUnitConverter.talonFX.sensorUnitsPer100msToRotationsPerMinute(sensorUnitsPer100ms);
    final var feetPerMinute = WHEEL_CONVERTER.rotationsToDistance(rotationsPerMinute);
    final var feetPerSecond = feetPerMinute / 60;

    return feetPerSecond;
  }
}
