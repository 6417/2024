// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Tankdrive_poseestimator;
import frc.robot.subsystems.drive.Controls;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.Tankdrive;

public class Robot extends TimedRobot {

    // Aliases for often used singleton instances
    DriveBase drive = Tankdrive.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    @Override
    public void robotInit() {
        // Add subsystems
        Shuffleboard.getTab("Drive").add(drive);
        Shuffleboard.getTab("Shooter").add(shooter);

        // Shuffleboard.getTab("Controls").add(Controls.getInstance()); // wtf nr. 2
        Shuffleboard.getTab("Debug").add(drive.getDefaultCommand());
        Shuffleboard.getTab("Debug").add(CommandScheduler.getInstance());

        // SignalLogger.setPath("/media/sda1");
        SignalLogger.setPath("/home/lvuser/logs");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Tankdrive_poseestimator.getInstance().updatePoseEstimator();

    }

    @Override
    public void teleopInit() {
        SignalLogger.setPath("test");
        shooter.setMotorSpeed(0);
    }

    // RamseteCommand command = null;
    @Override
    public void teleopPeriodic() {
        /*
         * if(Controls.joystick.getYButtonPressed()){
         * System.out.println("start command");
         * command = getAutonomousTrajectory.getInstance().start_command();
         * }
         * if (command != null){
         * System.out.println(CommandScheduler.getInstance().isScheduled(command));
         * }
         */

        if (Constants.Sysid.isTuning) {
            bindButtonsForSysid();
        } else {
            bindButtonsDefault();
            shooter.run(true);
        }

        // Brake mode
        if (Controls.joystick.getLeftBumperPressed()) {
            drive.release_brake();
        }
        if (Controls.joystick.getRightBumperPressed()) {
            drive.brake();
        }

        // System.out.println(Tankdrive_poseestimator.getInstance().m_poseEstimator.getEstimatedPosition());
    }

    private void bindButtonsDefault() {
        // Shooter
        if (Controls.joystick.getPOV() == 0) {
            shooter.changeMotorSpeed(1);
        } else if (Controls.joystick.getPOV() == 180) {
            shooter.changeMotorSpeed(-1);
        }
        if (Controls.joystick.getYButtonPressed()) {
            shooter.setMotorSpeed(Constants.Shooter.OptimalSpeakerSpeed);
        } else if (Controls.joystick.getXButtonPressed()) {
            shooter.setMotorSpeed(Constants.Shooter.OptimalAmpSpeed);
        } else if (Controls.joystick.getBButtonPressed()) {
            shooter.setMotorSpeed(0.0);
        }
    }

    private void bindButtonsForSysid() {
        if (Controls.joystick.getAButtonPressed()) {
            SignalLogger.start();
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).schedule();
            SignalLogger.start();
        } else if (Controls.joystick.getYButtonPressed()) {
            SignalLogger.start();
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).schedule();
            SignalLogger.start();
        } else if (Controls.joystick.getXButtonPressed()) {
            SignalLogger.start();
            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse).schedule();
            SignalLogger.start();
        } else if (Controls.joystick.getBButtonPressed()) {
            SignalLogger.start();
            drive.sysIdDynamic(SysIdRoutine.Direction.kForward).schedule();
            SignalLogger.start();
        } else if (Controls.joystick.getAButtonReleased() ||
                Controls.joystick.getBButtonReleased() ||
                Controls.joystick.getXButtonReleased() ||
                Controls.joystick.getYButtonReleased()) {
            SignalLogger.stop();
            CommandScheduler.getInstance().cancelAll();
            drive.brake();
            SignalLogger.stop();
        }
    }
}
