// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.vision_autonomous.Tankdrive_odometry;
import frc.robot.utils.CSVLogger;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class CSVLoggerCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CSVLogger loger;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CSVLoggerCommand(String pathname) {
    loger = new CSVLogger(pathname);
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loger.put("odometry_x", Tankdrive_odometry.getInstance().m_odometry.getPoseMeters().getX());
    loger.put("odometry_y", Tankdrive_odometry.getInstance().m_odometry.getPoseMeters().getY());
    loger.put("time", System.currentTimeMillis()/1000);
    //System.out.println(Tankdrive_odometry.getInstance().m_odometry.getPoseMeters().getX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    loger.writeToFile();
    loger.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
