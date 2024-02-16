package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerdriveAutoCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerdrivAuto m_subsystem;

  Timer timer = new Timer();

  public SwerdriveAutoCommand(SwerdriveAutoCommand subsystem) {
    swerdriveauto = subsystem;
    //addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    double t = timer.get();
    ChassisSpeeds speeds = swervderiveauto
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
