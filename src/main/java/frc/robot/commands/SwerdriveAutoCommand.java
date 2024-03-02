package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drive.swerve.SwerveDrive;
import frc.robot.subsystems.vision_autonomous.SwervdriveAuto;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerdriveAutoCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  Timer timer = new Timer();
  Trajectory trajectory;

  public SwerdriveAutoCommand(Trajectory tra) {
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
    ChassisSpeeds speeds = SwervdriveAuto.getInstance().getVelocitis(trajectory, t);
    SwerveDrive.getInstance().drive(speeds);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
