// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.Tankdrive;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  public Robot() {
  }

  @Override
  public void robotInit() {
  }

  DriveBase drive;

  @Override
  public void teleopInit(){
    drive = Tankdrive.getInstance();
  }

  @Override
  public void teleopPeriodic() {
    //System.out.println(CommandScheduler.getInstance().isScheduled(drive.getDefaultCommand()));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
