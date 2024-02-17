// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.fridowpi.motors.FridolinsMotor.PidType;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ClimberPid extends Command {

  // private Command[] commandliste = new
  // Command[Constants.Climber.anzFahrAbschnitte];

  // public void climberBedienen() {
  // var b = Constants.Climber.ausfahrBereich;
  // var n = Constants.Climber.anzFahrAbschnitte;
  // for (int i = 0; i < n; i++) {
  // commandliste[i] = new ClimberPid(b / n * i);
  // }
  // }

  private double target;

  public ClimberPid(double target) {
    this.target = target;
    // addRequirements(ClimberSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ClimberSubsystem.getInstance().seilZiehMotorLinks.setPidTarget(target, PidType.position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
