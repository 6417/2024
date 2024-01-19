package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveBase;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveBase m_subsystem;

  public DriveCommand(DriveBase subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    double y = Robot.joystick.getLeftY();
    double x = Robot.joystick.getLeftX();
    double rot = Robot.joystick.getRightX();
    m_subsystem.drive(y, x, rot);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
