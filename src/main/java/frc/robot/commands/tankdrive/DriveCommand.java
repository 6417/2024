package frc.robot.commands.tankdrive;

import frc.robot.Config;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.abstraction.baseClasses.BDrive;

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
    // double x = Config.joystick().getX();
    // double y = Config.joystick().getY();
    // double rot = Config.joystick().getTwist();
    // getRightY() somehow is always 0: Don't use
	var x = 0;
	var y = 0;
	var rot = 0;
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
