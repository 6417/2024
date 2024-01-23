// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Tankdrive_poseestimator;
import frc.robot.subsystems.drive.Controls;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.Tankdrive;
import frc.robot.subsystems.drive.getAutonomousTrajectory;

public class Robot extends TimedRobot {

    DriveBase drive;

    @Override
    public void robotInit() {
        Shuffleboard.getTab("Drive").add(Tankdrive.getInstance());
        // Shuffleboard.getTab("Controls").add(Controls.getInstance()); // wtf nr. 2
        Shuffleboard.getTab("Debug").add(Tankdrive.getInstance().getDefaultCommand());
        Shuffleboard.getTab("Debug").add(CommandScheduler.getInstance());
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
    RamseteCommand command = null;
    @Override
    public void teleopPeriodic() {
        if(Controls.joystick.getYButtonPressed()){
            System.out.println("start command");
            command = getAutonomousTrajectory.getInstance().start_command();
        }
        if (command != null){
            System.out.println(CommandScheduler.getInstance().isScheduled(command));
        }
        /* 
        if (Controls.joystick.getAButtonPressed()) {
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).schedule();
        } else if (Controls.joystick.getYButtonPressed()) {
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).schedule();
        } else if (Controls.joystick.getXButtonPressed()) {
            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse).schedule();
        } else if (Controls.joystick.getBButtonPressed()) {
            drive.sysIdDynamic(SysIdRoutine.Direction.kForward).schedule();
        }
        else if (
            Controls.joystick.getAButtonReleased() ||
            Controls.joystick.getBButtonReleased() ||
            Controls.joystick.getXButtonReleased() ||
            Controls.joystick.getYButtonReleased()
        ) {
            CommandScheduler.getInstance().cancelAll();
            // drive.brake();
        }
        */

        // if (Controls.joystick.getLeftBumperPressed()) {
        //     drive.release_brake();
        // }

        // System.out.println(Tankdrive_poseestimator.getInstance().m_poseEstimator.getEstimatedPosition());
    }
}
