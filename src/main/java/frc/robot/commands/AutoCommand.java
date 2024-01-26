package frc.robot.commands;

import frc.robot.subsystems.drive.getAutonomousTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;


public class AutoCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final getAutonomousTrajectory m_subsystem;
  RamseteCommand command;

  public AutoCommand(getAutonomousTrajectory subsystem) {
    m_subsystem = subsystem;
    command = m_subsystem.get_comand();
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    command.schedule();
  }

  @Override
  public void execute() {
    command.execute();
  }


  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}