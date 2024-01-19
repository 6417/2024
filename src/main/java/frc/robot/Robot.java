// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< HEAD
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
=======
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
>>>>>>> d59e0d7f142ca8955e2e11cabaa141c8bd124b1d
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.Tankdrive;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  DriveBase drive;
  final public static XboxController joystick = new XboxController(0);

  public Robot() {
  }

  @Override
  public void robotInit() {
    Tankdrive.getInstance();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    drive = Tankdrive.getInstance();
  }
  
  @Override
  public void teleopPeriodic() {
    if (joystick.getAButtonPressed()) {
      drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).schedule();
    } else if(joystick.getYButtonPressed()) {
      drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).schedule();
    } else if(joystick.getXButtonPressed()) {
      drive.sysIdDynamic(SysIdRoutine.Direction.kReverse).schedule();
    } else if(joystick.getBButtonPressed()) {
      drive.sysIdDynamic(SysIdRoutine.Direction.kForward).schedule();
    } 
  }
}
