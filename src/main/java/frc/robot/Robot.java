// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.abstraction.baseClasses.BShooter;
import frc.robot.subsystems.visionAutonomous.TankDrivePoseEstimator;

public class Robot extends TimedRobot {

	// Aliases for often used singleton instances
	AnalogEncoder absEncoder = new AnalogEncoder(0);
	Optional<BShooter> shooter = Config.active.getShooter();

	@Override
	public void robotInit() {
		// Add subsystems
		Shuffleboard.getTab("Drive").add(Config.drive());
		if (shooter.isPresent()) {
			Shuffleboard.getTab("Shooter").add(shooter.get());
		}

		// Shuffleboard.getTab("Controls").add(Controls.getInstance()); // wtf nr. 2
		Shuffleboard.getTab("Debug").add(Config.drive().getDefaultCommand());
		Shuffleboard.getTab("Debug").add(CommandScheduler.getInstance());
		Shuffleboard.getTab("Debug").addDouble("absEncoder", absEncoder::getAbsolutePosition);

		// Logger path for '.wpilog's
		SignalLogger.setPath("/media/sda1");
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		TankDrivePoseEstimator.getInstance().updatePoseEstimator();
	}

	@Override
	public void teleopInit() {
		SignalLogger.setPath("test");
		if (shooter.isPresent()) {
			shooter.get().setSpeedPercent(0);
		}
	}

	@Override
	public void teleopPeriodic() {
		// Button inputs
		if (Constants.Sysid.isTuning) {
			bindButtonsForSysid();
		} else {
			bindButtonsDefault();
			if (shooter.isPresent()) {
				shooter.get().run(true);
			}
		}

		// Set brake mode
		if (Controls.joystick.getLeftBumperPressed()) {
			Config.drive().release_brake();
		}
		if (Controls.joystick.getRightBumperPressed()) {
			Config.drive().brake();
		}
	}

	private void bindButtonsDefault() {
		// Shooter
		if (shooter.isPresent()) {
			var s = shooter.get();
			if (Controls.joystick.getPOV() == 0) {
				s.setSpeedPercent(
						s.getSpeedPercent() + 0.1);
			} else if (Controls.joystick.getPOV() == 180) {
				s.setSpeedPercent(
						s.getSpeedPercent() - 0.1);
			}
			if (Controls.joystick.getYButtonPressed()) {
				s.setSpeedPercent(Constants.Shooter.OptimalSpeakerSpeed);
			} else if (Controls.joystick.getXButtonPressed()) {
				s.setSpeedPercent(Constants.Shooter.OptimalAmpSpeed);
			} else if (Controls.joystick.getBButtonPressed()) {
				s.setSpeedPercent(0.0);
			} else if (Controls.joystick.getAButtonPressed()) {
				s.setSpeedPercent(1.0);
			}
		}
	}

	private void bindButtonsForSysid() {
		if (Controls.joystick.getAButtonPressed()) {
			SignalLogger.start();
			Config.drive().sysIdQuasistatic(SysIdRoutine.Direction.kReverse).schedule();
			SignalLogger.start();
		} else if (Controls.joystick.getYButtonPressed()) {
			SignalLogger.start();
			Config.drive().sysIdQuasistatic(SysIdRoutine.Direction.kForward).schedule();
			SignalLogger.start();
		} else if (Controls.joystick.getXButtonPressed()) {
			SignalLogger.start();
			Config.drive().sysIdDynamic(SysIdRoutine.Direction.kReverse).schedule();
			SignalLogger.start();
		} else if (Controls.joystick.getBButtonPressed()) {
			SignalLogger.start();
			Config.drive().sysIdDynamic(SysIdRoutine.Direction.kForward).schedule();
			SignalLogger.start();
		} else if (Controls.joystick.getAButtonReleased() ||
				Controls.joystick.getBButtonReleased() ||
				Controls.joystick.getXButtonReleased() ||
				Controls.joystick.getYButtonReleased()) {
			SignalLogger.stop();
			CommandScheduler.getInstance().cancelAll();
			Config.drive().brake();
			SignalLogger.stop();
		}
	}
}
