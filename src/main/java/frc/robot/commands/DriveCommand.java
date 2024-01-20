package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.drive.Controls;
import frc.robot.subsystems.drive.DriveBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveBase m_subsystem;

  public DriveCommand(DriveBase subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() { }

  @Override
  public void execute() {
    double x = Controls.joystick.getLeftX();
    double y = Controls.joystick.getLeftY();
    double rot = Controls.joystick.getRightX();
    // getRightY() somehow is always 0: Don't use
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
