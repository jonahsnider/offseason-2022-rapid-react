// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.misc.util.CtreModuleState;
import frc.robot.swerve.drive.SwerveModuleDrive;
import frc.robot.swerve.steer.SwerveModuleSteer;

public class SwerveModule {
  private final SwerveModuleSteer steer;
  private final SwerveModuleDrive drive;

  public SwerveModule(
      SwerveModuleConstants constants, TalonFX driveMotor, TalonFX steerMotor, CANCoder encoder) {
    this.steer = new SwerveModuleSteer(constants, steerMotor, encoder);
    this.drive = new SwerveModuleDrive(driveMotor);
  }

  public void setDesiredState(SwerveModuleState state) {
    final var steerMotorAngle = steer.getSteerAngle();

    state = CtreModuleState.optimize(state, steerMotorAngle);

    steer.setGoalPosition(state.angle);
    drive.setGoalVelocity(state.speedMetersPerSecond);
  }

  public SwerveModuleState getState() {
    final var steerMotorAngle = steer.getSteerAngle();
    final var driveMotorVelocity = drive.getVelocity();

    return new SwerveModuleState(driveMotorVelocity, steerMotorAngle);
  }
}
