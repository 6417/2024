// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.fridowpi.motors.FridolinsMotor.PidType;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberPid extends Command {
	private ClimberSubsystem subsystem;

  private double target;

  public ClimberPid(ClimberSubsystem subsystem, double target) {
    this.target = target;
	this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.seilZiehMotorLinks.setPidTarget(target, PidType.position);
    subsystem.seilZiehMotorRechts.setPidTarget(target, PidType.position);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
	return subsystem.seilZiehMotorLinks.pidAtTarget() && subsystem.seilZiehMotorRechts.pidAtTarget();
  }
}
