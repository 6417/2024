// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.fridowpi.sensors.FridoNavx;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.vision_autonomous.Tankdrive_odometry;
import frc.robot.utils.CSVLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class CSVLoggerCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final CSVLogger loger;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  Trajectory trajectory;
  Timer t;

  public CSVLoggerCommand(String pathname, Trajectory trajectory) {
    loger = new CSVLogger(pathname);
    this.trajectory = trajectory;

    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    t = new Timer();
    t.start();
  }

  int i = 0;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (t.hasElapsed(0.1 * i)) {
      var goal = trajectory.sample(t.get());
      loger.put("odometry_x", Tankdrive_odometry.getInstance().m_odometry.getEstimatedPosition().getX());
      loger.put("odometry_y", Tankdrive_odometry.getInstance().m_odometry.getEstimatedPosition().getY());
      loger.put("pose_traj_x", goal.poseMeters.getX());
      loger.put("pose_traj_y", goal.poseMeters.getY());
      loger.put("rot_traj", goal.poseMeters.getRotation().getDegrees());
      loger.put("rot_gyro", FridoNavx.getInstance().getRotation2d().getDegrees());
      loger.put("time", t.get());
      loger.put("rot_odo",
          Tankdrive_odometry.getInstance().m_odometry.getEstimatedPosition().getRotation().getDegrees());
      // System.out.println(Tankdrive_odometry.getInstance().m_odometry.getPoseMeters().getX());
      i++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    loger.writeToFile();
    loger.close();
    t.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
