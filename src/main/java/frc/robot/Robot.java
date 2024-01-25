// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.vision_autonomous.Tankdrive_poseestimator;
import frc.robot.subsystems.drive.Controls;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.Tankdrive;
import frc.robot.subsystems.drive.getAutonomousTrajectory;

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

    CANSparkMax spark = new CANSparkMax(1, MotorType.kBrushless);

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

    RamseteCommand aCommand = null;
    @Override
    public void teleopPeriodic() {
        //spark.set(0.1);
        Tankdrive.getInstance().differentialDrive.feed();
        if(Controls.joystick.getYButtonPressed()){
            System.out.println("start command");
            //AutoCommand aComand = new AutoCommand(getAutonomousTrajectory.getInstance());
            //aComand.schedule();
            getAutonomousTrajectory.getInstance().get_comand().schedule();
            //command = getAutonomousTrajectory.getInstance().start_command();
            //getAutonomousTrajectory.getInstance();
        }
        if (aCommand != null){
            System.out.println(CommandScheduler.getInstance().isScheduled(aCommand));
        }
        if(Controls.joystick.getLeftBumperPressed()){
            Tankdrive.getInstance().brake();
        }
        if (Controls.joystick.getRightBumperPressed()){
            Tankdrive.getInstance().release_brake();
        }
        
        /* 
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
        */
    }
}
