package frc.robot.commands;

import frc.robot.subsystems.visionAutonomous.SwervedriveAuto;

import org.apache.logging.log4j.core.lookup.SystemPropertiesLookup;

import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Config;
import frc.robot.Controls;
import frc.robot.abstraction.baseClasses.BDrive.SpeedFactor;

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
  Rotation2d desiredendRot;

  public SwervedriveAutoCommand(Trajectory tra, Rotation2d endRot) {
    trajectory = tra;
    System.out.println(trajectory.toString());
    mode = autoDriveMode.trajectory;
    desiredendRot = endRot;
    // addRequirements(subsystem);
  }

  public SwervedriveAutoCommand(Pose2d pose, Rotation2d endRot) {
    this.pose = pose;
    mode = autoDriveMode.pose;
    desiredendRot = endRot;
  }

  @Override
  public void initialize() {
    timer.start();
    Controls.setActiveSpeedFactor(SpeedFactor.FAST);
  }

  @Override
  public void execute() {
    // man kann die trajectory brauchen, weill der holonomic die punkte ankorigiert
    // welche die
    // spline zurückgibt, aber eventuell stören die velocities der trajectory mit
    // d
    if (mode == autoDriveMode.trajectory) {
      double t = timer.get();
      speeds = Config.active.getAuto().get().getVelocitiesAtTimepoint(trajectory, t, desiredendRot);
      Config.drive().drive(speeds);

    } else if (mode == autoDriveMode.pose) {
      speeds = ((SwervedriveAuto)Config.active.getAuto().get()).getVelocitiesToPose(pose, desiredendRot);
      Config.drive().drive(speeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Controls.setActiveSpeedFactor(SpeedFactor.DEFAULT_SPEED);
  }

  @Override

  public boolean isFinished() {
    // fist idea to stop command, not realy working, stop to eraly
    //return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    return timer.get() > 2.0;
    /*
    if (Math.hypot(Config.drive().getPos().getX()
        - trajectory.getStates().get(trajectory.getStates().size()-1).poseMeters.getX(),
        Config.drive().getPos().getY() -
            trajectory.getStates().get(trajectory.getStates().size()-1).poseMeters.getY()) < 0.2) {
              return true;
    } else{
      return false;
    }
    */

    // // second attempt to sotp command, guest values, not now if it works
    // if (Math.abs(speeds.vxMetersPerSecond) <= 0.1 &&
    // Math.abs(speeds.vyMetersPerSecond) <= 0.1 &&
    // Math.abs(speeds.omegaRadiansPerSecond) <= 0.05) {
    // if (timer.get() >= 1) {
    // return true;
    // }
    // }

    // return false;
  }
}
