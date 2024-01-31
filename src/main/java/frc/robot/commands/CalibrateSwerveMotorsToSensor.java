// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class CalibrateSwerveMotorsToSensor extends Command {

  private double angle;

  public CalibrateSwerveMotorsToSensor() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angle = 0;
    DriveSubsystem.getInstance().enableRotationLimitSwitch(true);
    DriveSubsystem.getInstance().setAllAngleEncodersToZero();
    DriveSubsystem.getInstance().stopAllAngleMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle += 1;
    DriveSubsystem.getInstance().setModuleAngle(angle, angle, angle, angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.getInstance().stopAllAngleMotors();
    DriveSubsystem.getInstance().setAllAngleEncodersToZero();
    DriveSubsystem.getInstance().enableRotationLimitSwitch(false);
  }

  // Returns true when the command should end.

  @Override
  public boolean isFinished() {
    return DriveSubsystem.getInstance().allRotationLimitswitchesActive();
  }
}
