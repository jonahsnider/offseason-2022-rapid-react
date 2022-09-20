// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve.steer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.misc.util.GearingConverter;
import frc.robot.misc.util.sensors.SensorUnitConverter;
import frc.robot.swerve.SwerveModuleConstants;

public class SwerveModuleSteer {
  private static final GearingConverter GEARING_CONVERTER = GearingConverter.fromReduction(10);

  private final TalonFX motor;
  private final CANCoder encoder;
  private final SwerveModuleConstants constants;

  /** Creates a new SwerveModuleSteer. */
  public SwerveModuleSteer(SwerveModuleConstants constants, TalonFX motor, CANCoder encoder) {
    this.constants = constants;
    this.motor = motor;
    this.encoder = encoder;

    motor.config_kP(0, 0);
    motor.config_kI(0, 0);
    motor.config_kD(0, 0);
    motor.config_kF(0, 0);

    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));

    resetMotorPosition();
  }

  public void setGoalPosition(Rotation2d angle) {
    final var rotations = Units.radiansToRotations(angle.getRadians());
    final var rotationsBeforeGearing = GEARING_CONVERTER.afterToBeforeGearing(rotations);
    final var sensorUnitsBeforeGearing =
        SensorUnitConverter.talonFX.rotationsToSensorUnits(rotationsBeforeGearing);

    motor.set(ControlMode.Position, sensorUnitsBeforeGearing);
  }

  public Rotation2d getSteerAngle() {
    final var degrees = encoder.getAbsolutePosition();

    return Rotation2d.fromDegrees(degrees);
  }

  private void resetMotorPosition() {
    final var absolutePosition =
        Units.radiansToRotations(
            Rotation2d.fromDegrees(encoder.getAbsolutePosition())
                .minus(constants.angleOffset)
                .getRadians());
    final var absolutePositionBeforeGearing =
        GEARING_CONVERTER.afterToBeforeGearing(absolutePosition);
    final var sensorUnitsBeforeGearing =
        SensorUnitConverter.talonFX.rotationsToSensorUnits(absolutePositionBeforeGearing);
    motor.setSelectedSensorPosition(sensorUnitsBeforeGearing);
  }
}
