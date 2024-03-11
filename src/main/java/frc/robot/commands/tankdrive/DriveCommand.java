package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.abstraction.baseClasses.BDrive;
import frc.robot.joystick.Joystick2024;

public class DriveCommand extends Command {

  private final BDrive m_subsystem;

  public DriveCommand(BDrive subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() { }

  @Override
  public void execute() {
    double y = Joystick2024.getInstance().getPrimaryJoystick().getY();
    double rot = Joystick2024.getInstance().getPrimaryJoystick().getTwist();
    m_subsystem.drive(y, 0, rot);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
