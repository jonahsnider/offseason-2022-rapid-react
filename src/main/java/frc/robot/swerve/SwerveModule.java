// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.misc.util.CircleConverter;
import frc.robot.misc.util.CtreModuleState;
import frc.robot.misc.util.GearingConverter;
import frc.robot.misc.util.sensors.SensorUnitConverter;

public class SwerveModule {
  private static final SimpleMotorFeedforward DRIVE_MOTOR_FEEDFORWARD =
      new SimpleMotorFeedforward(0, 0);
  private static final double DRIVE_MOTOR_MAX_VOLTAGE = 12;
  private static final GearingConverter DRIVE_MOTOR_GEARING_CONVERTER =
      GearingConverter.fromReduction(10);
  private static final GearingConverter STEER_MOTOR_GEARING_CONVERTER =
      GearingConverter.fromReduction(10);
  private static final CircleConverter DRIVE_MOTOR_WHEEL_CONVERTER =
      CircleConverter.fromDiameter(6);

  private final TalonFX driveMotor;
  private final CANCoder encoder;
  private final TalonFX steerMotor;
  private final SwerveModuleConstants constants;

  public SwerveModule(
      SwerveModuleConstants constants, TalonFX driveMotor, TalonFX steerMotor, CANCoder encoder) {
    this.constants = constants;
    this.driveMotor = driveMotor;
    this.steerMotor = steerMotor;
    this.encoder = encoder;

    driveMotor.config_kP(0, 0);
    driveMotor.config_kI(0, 0);
    driveMotor.config_kD(0, 0);
    driveMotor.config_kF(0, 0);
    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));

    steerMotor.config_kP(0, 0);
    steerMotor.config_kI(0, 0);
    steerMotor.config_kD(0, 0);
    steerMotor.config_kF(0, 0);
    steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));

    resetSteerMotorPosition();
  }

  public void setDesiredState(SwerveModuleState state) {
    final var steerMotorPosition = getSteerMotorPosition();

    state = CtreModuleState.optimize(state, steerMotorPosition);

    final var rotations = Units.radiansToRotations(state.angle.getRadians());
    final var rotationsBeforeGearing =
        STEER_MOTOR_GEARING_CONVERTER.afterToBeforeGearing(rotations);
    final var sensorUnitsBeforeGearing =
        SensorUnitConverter.talonFX.rotationsToSensorUnits(rotationsBeforeGearing);
    steerMotor.set(ControlMode.Position, sensorUnitsBeforeGearing);

    final var feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);
    final var feetPerMinute = feetPerSecond * 60;
    final var rotationsPerMinute = DRIVE_MOTOR_WHEEL_CONVERTER.distanceToRotations(feetPerMinute);
    final var sensorUnitsPer100ms =
        SensorUnitConverter.talonFX.rotationsPerMinuteToSensorUnitsPer100ms(rotationsPerMinute);
    final var sensorUnitsPer100msBeforeGearing =
        DRIVE_MOTOR_GEARING_CONVERTER.afterToBeforeGearing(sensorUnitsPer100ms);
    driveMotor.set(
        ControlMode.Velocity,
        sensorUnitsPer100msBeforeGearing,
        DemandType.ArbitraryFeedForward,
        DRIVE_MOTOR_FEEDFORWARD.kv / DRIVE_MOTOR_MAX_VOLTAGE);
  }

  public SwerveModuleState getState() {
    final var steerMotorPosition = getSteerMotorPosition();
    final var driveMotorVelocity = getDriveMotorVelocity();

    return new SwerveModuleState(driveMotorVelocity, steerMotorPosition);
  }

  private Rotation2d getSteerMotorPosition() {
    final var degrees = encoder.getAbsolutePosition();

    return Rotation2d.fromDegrees(degrees);
  }

  private double getDriveMotorVelocity() {
    final var sensorUnitsPer100msBeforeGearing = driveMotor.getSelectedSensorVelocity();
    final var sensorUnitsPer100ms =
        DRIVE_MOTOR_GEARING_CONVERTER.beforeToAfterGearing(sensorUnitsPer100msBeforeGearing);
    final var rotationsPerMinute =
        SensorUnitConverter.talonFX.sensorUnitsPer100msToRotationsPerMinute(sensorUnitsPer100ms);
    final var feetPerMinute = DRIVE_MOTOR_WHEEL_CONVERTER.rotationsToDistance(rotationsPerMinute);
    final var feetPerSecond = feetPerMinute / 60;

    return feetPerSecond;
  }

  private void resetSteerMotorPosition() {
    final var absolutePosition =
        Units.radiansToRotations(
            Rotation2d.fromDegrees(encoder.getAbsolutePosition())
                .minus(constants.angleOffset)
                .getRadians());
    final var absolutePositionBeforeGearing =
        STEER_MOTOR_GEARING_CONVERTER.afterToBeforeGearing(absolutePosition);
    final var sensorUnitsBeforeGearing =
        SensorUnitConverter.talonFX.rotationsToSensorUnits(absolutePositionBeforeGearing);
    steerMotor.setSelectedSensorPosition(sensorUnitsBeforeGearing);
  }
}
