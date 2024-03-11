package frc.robot.commands;

import frc.robot.subsystems.visionAutonomous.SwervedriveAuto;

import org.apache.logging.log4j.core.lookup.SystemPropertiesLookup;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Config;

public class SwervedriveAutoCommand extends Command {

	Timer timer = new Timer();
	Trajectory trajectory;
  Timer timer = new Timer();
  Trajectory trajectory;
  ChassisSpeeds speeds;

  public SwervedriveAutoCommand(Trajectory tra) {
    trajectory = tra;
    // addRequirements(subsystem);
  }

	@Override
	public void initialize() {
		timer.start();
	}

  @Override
  public void execute() {
    double t = timer.get();
    speeds = SwervedriveAuto.getInstance().getVelocities(trajectory, t);
    Config.drive().drive(speeds);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    // fist idea to stop command, not realy working, stop to eraly
    // return timer.hasElapsed(trajectory.getTotalTimeSeconds());

    // second attempt to sotp command, guest values, not now if it works
    if (Math.abs(speeds.vxMetersPerSecond) <= 0.1 &&
        Math.abs(speeds.vyMetersPerSecond) <= 0.1 &&
        Math.abs(speeds.omegaRadiansPerSecond) <= 0.05) {
      if (timer.get() >= 1) {
        return true;
      }
    }

    return false;
  }
}
