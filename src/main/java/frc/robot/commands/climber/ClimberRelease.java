// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ClimberRelease extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_subsystem;
  private double winkel = Constants.Climber.maxServoPos;

  
  public ClimberRelease(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (winkel == Constants.Climber.maxServoPos){
      winkel = 0; 
    }
    else{
      winkel = Constants.Climber.maxServoPos;
    }
    m_subsystem.federLoslassMotorLinks.setAngle(winkel);
    System.out.println("Value: " + m_subsystem.federLoslassMotorLinks.getAngle());
    
    // m_subsystem.federLoslassMotorLinks.setAngle(m_subsystem.federLoslassMotorLinks.getAngle() + 180);
    //m_subsystem.federLoslassMotorRechts.setAngle(m_subsystem.federLoslassMotorRechts.getAngle() + 180);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
