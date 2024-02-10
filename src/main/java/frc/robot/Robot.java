// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.joystick.Joystick2024;
import frc.robot.joystick.IdsWithState.State;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.vision_autonomous.Tankdrive_odometry;
import frc.robot.subsystems.vision_autonomous.Tankdrive_poseestimator;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.Tankdrive;

public class Robot extends TimedRobot {

    // Aliases for often used singleton instances
    DriveBase drive = Tankdrive.getInstance();
    // ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ShuffleboardTab tab;

    @Override
    public void robotInit() {
        // Add subsystems
        Shuffleboard.getTab("Drive").add(drive);
        // Shuffleboard.getTab("Shooter").add(shooter);

        // Shuffleboard.getTab("Controls").add(Controls.getInstance()); // wtf nr. 2
        Shuffleboard.getTab("Debug").add(drive.getDefaultCommand());
        Shuffleboard.getTab("Debug").add(CommandScheduler.getInstance());

        // SignalLogger.setPath("/media/sda1");
        SignalLogger.setPath("/home/lvuser/logs");

        tab = Shuffleboard.getTab("robotpos");

        Joystick2024.getInstance().setup(State.SYSID_TUNING);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Tankdrive_poseestimator.getInstance().updatePoseEstimator();
        Tankdrive_odometry.getInstance().update_robot_pose();
    }

    @Override
    public void teleopInit() {
        SignalLogger.setPath("test");
        // shooter.setMotorSpeed(0);
    }

    @Override
    public void teleopPeriodic() {

        // if (Controls.joystick.getRawButtonPressed(4)) {
        //     Tankdrive.getInstance().brake();
        // }
        // if (Controls.joystick.getRawButtonPressed(5)) {
        //     Tankdrive.getInstance().release_brake();
        // }

        // if (Constants.Sysid.isTuning) {
        Joystick2024.getInstance().setState(State.SYSID_TUNING);
        // } else {
        //     bindButtonsDefault();
        //     shooter.run(true);
        // }

        // // Brake mode
        // if (Controls.joystick.getLeftBumperPressed()) {
        //     drive.release_brake();
        // }
        // if (Controls.joystick.getRightBumperPressed()) {
        //     drive.brake();
        // }
    }

}
