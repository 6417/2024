// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SimplePrintCommand extends Command {
  String msg;

  public SimplePrintCommand(String msg) {
    this.msg = msg;
  }
  
  @Override
  public void initialize() {
    System.out.println("<DebugPrint>" + msg);
  }
  
  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
