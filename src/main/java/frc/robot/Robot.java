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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.vision_autonomous.Gyro;
import frc.robot.subsystems.vision_autonomous.Tankdrive_odometry;
import frc.robot.subsystems.vision_autonomous.Tankdrive_poseestimator;
import frc.robot.subsystems.drive.Controls;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.Tankdrive;
import frc.robot.subsystems.drive.getAutonomousTrajectory;

public class Robot extends TimedRobot {

    // Aliases for often used singleton instances
    DriveBase drive = Tankdrive.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ShuffleboardTab tab;
    private boolean 
    
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

        tab = Shuffleboard.getTab("robotpos");

        JoystickButton resetButton = new JoystickButton(Controls.joystick,Logitech.b.getButtonId());
        resetButton.onTrue(new InstantCommand(()->{Gyro.getInstance().reset();
        Tankdrive_odometry.getInstance().reset_odometry();}));
    }

    CANSparkMax spark = new CANSparkMax(1, MotorType.kBrushless);

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Tankdrive_poseestimator.getInstance().updatePoseEstimator();
        Tankdrive_odometry.getInstance().update_robot_pose();
    }

    @Override
    public void teleopInit() {
        SignalLogger.setPath("test");
        shooter.setMotorSpeed(0);
    }

    //AutoCommand aComand = null;
    Command auto_command = null;

    @Override
    public void teleopPeriodic() {
        // spark.set(0.1);
        Tankdrive.getInstance().differentialDrive.feed();
        if (Controls.joystick.getRawButtonPressed(0)) {
            System.out.println("start command");
            //aComand = new AutoCommand(getAutonomousTrajectory.getInstance());
            //aComand.schedule();
            auto_command = getAutonomousTrajectory.getInstance().start_command();
            //Trajectory path = getAutonomousTrajectory.getInstance().get_raw_trajectory();
            //oBCommand command = new oBCommand(Own_trajectory_generytor.getInstance());
            //command.schedule();
            //Tankdrive_poseestimator.getInstance().m_poseEstimator.resetPosition(new Rotation2d(0, 0),
                    //Tankdrive.getInstance().getWeelPosition(), new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
            //getAutonomousTrajectory.getInstance().get_comand().schedule();
            //getAutonomousTrajectory.getInstance().start_command();
            // getAutonomousTrajectory.getInstance();
            //Tankdrive.getInstance().setVolts(2,2);
        }
        //System.out.println(Tankdrive_poseestimator.getInstance().m_poseEstimator.getEstimatedPosition());
        if (auto_command != null){
            //System.out.println(CommandScheduler.getInstance().isScheduled(auto_command));
        }
        
        if (Controls.joystick.getRawButtonPressed(4)) {
            Tankdrive.getInstance().brake();
        }
        if (Controls.joystick.getRawButtonPressed(5)) {
            Tankdrive.getInstance().release_brake();
        }

        if(Controls.joystick.trigger(null))
        if (Constants.Climber.climberButtonsEnabled){

        }

        /*
         * if (Constants.Sysid.isTuning) {
         * bindButtonsForSysid();
         * } else {
         * bindButtonsDefault();
         * shooter.run(true);
         * }
         * 
         * // Brake mode
         * if (Controls.joystick.getLeftBumperPressed()) {
         * drive.release_brake();
         * }
         * if (Controls.joystick.getRightBumperPressed()) {
         * drive.brake();
         * }
         * 
         * // System.out.println(Tankdrive_poseestimator.getInstance().m_poseEstimator.
         * getEstimatedPosition());
         * }
         * 
         * private void bindButtonsDefault() {
         * // Shooter
         * if (Controls.joystick.getPOV() == 0) {
         * shooter.changeMotorSpeed(1);
         * } else if (Controls.joystick.getPOV() == 180) {
         * shooter.changeMotorSpeed(-1);
         * }
         * if (Controls.joystick.getYButtonPressed()) {
         * shooter.setMotorSpeed(Constants.Shooter.OptimalSpeakerSpeed);
         * } else if (Controls.joystick.getXButtonPressed()) {
         * shooter.setMotorSpeed(Constants.Shooter.OptimalAmpSpeed);
         * } else if (Controls.joystick.getBButtonPressed()) {
         * shooter.setMotorSpeed(0.0);
         * }
         * }
         * 
         * private void bindButtonsForSysid() {
         * if (Controls.joystick.getAButtonPressed()) {
         * SignalLogger.start();
         * drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).schedule();
         * SignalLogger.start();
         * } else if (Controls.joystick.getYButtonPressed()) {
         * SignalLogger.start();
         * drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).schedule();
         * SignalLogger.start();
         * } else if (Controls.joystick.getXButtonPressed()) {
         * SignalLogger.start();
         * drive.sysIdDynamic(SysIdRoutine.Direction.kReverse).schedule();
         * SignalLogger.start();
         * } else if (Controls.joystick.getBButtonPressed()) {
         * SignalLogger.start();
         * drive.sysIdDynamic(SysIdRoutine.Direction.kForward).schedule();
         * SignalLogger.start();
         * } else if (Controls.joystick.getAButtonReleased() ||
         * Controls.joystick.getBButtonReleased() ||
         * Controls.joystick.getXButtonReleased() ||
         * Controls.joystick.getYButtonReleased()) {
         * SignalLogger.stop();
         * CommandScheduler.getInstance().cancelAll();
         * drive.brake();
         * SignalLogger.stop();
         * }
         */
    }
}
