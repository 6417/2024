// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Tankdrive_poseestimator;
import frc.robot.subsystems.drive.Controls;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.Tankdrive;

public class Robot extends TimedRobot {

    DriveBase drive;
    private ShooterSubsystem shooter;

    @Override
    public void robotInit() {
        Shuffleboard.getTab("Drive").add(Tankdrive.getInstance());
        // Shuffleboard.getTab("Controls").add(Controls.getInstance()); // wtf nr. 2
        Shuffleboard.getTab("Debug").add(Tankdrive.getInstance().getDefaultCommand());
        Shuffleboard.getTab("Debug").add(CommandScheduler.getInstance());
        SignalLogger.setPath("/media/sda1");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Tankdrive_poseestimator.getInstance().updatePoseEstimator();

    }

    @Override
    public void teleopInit() {
        drive = Tankdrive.getInstance();
    }

    //RamseteCommand command = null;
    @Override
    public void teleopPeriodic() {
        /* 
        if(Controls.joystick.getYButtonPressed()){
            System.out.println("start command");
            command = getAutonomousTrajectory.getInstance().start_command();
        }
        if (command != null){
            System.out.println(CommandScheduler.getInstance().isScheduled(command));
        }
        */
         
        if (Controls.joystick.getAButtonPressed()) {
            SignalLogger.start();
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).schedule();
        } else if (Controls.joystick.getYButtonPressed()) {
            SignalLogger.start();
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).schedule();
        } else if (Controls.joystick.getXButtonPressed()) {
            SignalLogger.start();
            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse).schedule();
        } else if (Controls.joystick.getBButtonPressed()) {
            SignalLogger.start();
            drive.sysIdDynamic(SysIdRoutine.Direction.kForward).schedule();
        }
        else if (
            Controls.joystick.getAButtonReleased() ||
            Controls.joystick.getBButtonReleased() ||
            Controls.joystick.getXButtonReleased() ||
            Controls.joystick.getYButtonReleased()
        ) {
            SignalLogger.stop();
            CommandScheduler.getInstance().cancelAll();
            drive.brake();
        }
        if (Controls.joystick.getLeftBumperPressed()) {
            drive.release_brake();
        }
        if (Controls.joystick.getRightBumperPressed()){
            drive.brake();
        }

        // Shooter
        if (Controls.joystick.getPOV() == 0) {
            shooter.changeMotorSpeed(10);
        } else if (Controls.joystick.getPOV() == 180) {
            shooter.changeMotorSpeed(-10);
        }

        // System.out.println(Tankdrive_poseestimator.getInstance().m_poseEstimator.getEstimatedPosition());
    }
}
