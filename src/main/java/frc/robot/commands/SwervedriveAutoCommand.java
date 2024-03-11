package frc.robot.commands;

import frc.robot.subsystems.visionAutonomous.SwervedriveAuto;

import org.apache.logging.log4j.core.lookup.SystemPropertiesLookup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Config;

public class SwervedriveAutoCommand extends Command {

  private enum autoDriveMode {
    trajectory,
    pose
  }

  Timer timer = new Timer();
  Trajectory trajectory;
  Pose2d pose;
  autoDriveMode mode;
  ChassisSpeeds speeds;

  public SwervedriveAutoCommand(Trajectory tra) {
    trajectory = tra;
    mode = autoDriveMode.trajectory;
    // addRequirements(subsystem);
  }

  public SwervedriveAutoCommand(Pose2d pose) {
    this.pose = pose;
    mode = autoDriveMode.pose;
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    //man kann die trajectory brauchen, weill der holonomic die punkte ankorigiert welche die
    //spline zurückgibt, aber eventuell stören die velocities der trajectory mit
    //d
    if (mode == autoDriveMode.trajectory) {
      double t = timer.get();
      speeds = SwervedriveAuto.getInstance().getVelocities(trajectory, t);
      Config.drive().drive(speeds);
    } else if (mode == autoDriveMode.pose){
      double t = timer.get();
      speeds = SwervedriveAuto.getInstance().getVelocities(pose, t);
      Config.drive().drive(speeds);
    }
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
