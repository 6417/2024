package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drive.DriveBase;

import java.sql.PseudoColumnUsage;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveBase m_subsystem;

  private PS4Controller joystick = new PS4Controller(0);


  public DriveCommand(DriveBase subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_subsystem.drive(
      joystick.getLeftY(),
      joystick.getLeftX(),
      joystick.getRightX());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
