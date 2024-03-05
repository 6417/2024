package frc.robot.commands;

import frc.robot.subsystems.visionAutonomous.SwervedriveAuto;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Config;

public class SwervedriveAutoCommand extends Command {

  Timer timer = new Timer();
  Trajectory trajectory;

  public SwervedriveAutoCommand(Trajectory tra) {
    trajectory = tra;
    //addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    double t = timer.get();
    ChassisSpeeds speeds = SwervedriveAuto.getInstance().getVelocities(trajectory, t);
    Config.drive().drive(speeds);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    //return timer.hasElapsed(trajectory.getTotalTimeSeconds());
	return false;
  }
}
