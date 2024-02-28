// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.fridowpi.command.FridoCommand;
import frc.fridowpi.command.ParallelCommandGroup;
import frc.fridowpi.command.SequentialCommandGroup;
import frc.fridowpi.motors.FridoCanSparkMax;
import frc.fridowpi.motors.FridoServoMotor;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.FridolinsMotor.LimitSwitchPolarity;
import frc.fridowpi.motors.FridolinsMotor.PidType;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BClimber;

public class ClimberSubsystem extends BClimber {
	private FridolinsMotor seilMotorLinks = new FridoCanSparkMax(Constants.Climber.seilZiehMotorLinks, MotorType.kBrushless);
	private FridolinsMotor seilMotorRechts = new FridoCanSparkMax(Constants.Climber.seilZiehMotorRechts, MotorType.kBrushless);
	private FridoServoMotor servoLinks = new FridoServoMotor(Constants.Climber.servoLinksId);
	private FridoServoMotor servoRechts = new FridoServoMotor(Constants.Climber.servoRechtsId);

	/** Creates a new ClimberSubsystem. */
	public ClimberSubsystem() {
	}

	@Override
	public void init() {
		seilMotorLinks.factoryDefault();
		seilMotorRechts.factoryDefault();

		seilMotorLinks.setPID(Constants.Climber.pidValuesSlot0);
		seilMotorRechts.setPID(Constants.Climber.pidValuesSlot0);
		seilMotorLinks.enableForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen, true);
		seilMotorRechts.enableForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen, true);
		seilMotorRechts.follow(seilMotorLinks, DirectionType.followMaster);
		seilMotorLinks.setIdleMode(IdleMode.kBrake);
		servoLinks.setBoundsMicroseconds(2200, 1499, 1500, 1501, 800);
		servoLinks.setMaxAngle(130);
	}

	@Override
	public void run() {
	}

	@Override
	public void release() {
		new SequentialCommandGroup(
				new ParallelCommandGroup(
						new InstantCommand(
								() -> this.servoLinks.setAngle(Constants.Climber.maxServoPos)),
						new InstantCommand(
								() -> this.servoRechts.setAngle(Constants.Climber.maxServoPos))),
				new ClimberPid(this, Constants.Climber.ausfahrBereich)).schedule();
	}

	@Override
	public void retract() {
		new ClimberPid(this, Constants.Climber.zielPosition).schedule();
	}

	public class ClimberPid extends FridoCommand {
		private ClimberSubsystem subsystem;
		private double target;

		public ClimberPid(ClimberSubsystem subsystem, double target) {
			if (target > Constants.Climber.ausfahrBereich) {
				target = Constants.Climber.ausfahrBereich;
				System.err.println("<ClimberSubsystem::ClimberPid> ausfahrBereich ueberschritten");
			}
			if (target < Constants.Climber.minimumAusfahrBereich) {
				target = Constants.Climber.minimumAusfahrBereich;
				System.err.println("<ClimberSubsystem::ClimberPid> minimaler ausfahrBereich ueberschritten");
			}
			this.target = target;
			this.subsystem = subsystem;
			addRequirements(subsystem);
		}

		@Override
		public void initialize() {
			subsystem.seilMotorLinks.setPidTarget(target, PidType.position);
			subsystem.seilMotorRechts.setPidTarget(target, PidType.position);
			Config.data().drive().motorIds();
		}

		@Override
		public boolean isFinished() {
			return subsystem.seilMotorLinks.pidAtTarget() && subsystem.seilMotorRechts.pidAtTarget();
		}
	}

	@Override
	public void stop() {
		seilMotorLinks.stopMotor();
		seilMotorRechts.stopMotor();
	}

	@Override
	public ClimberData getData() {
		var motorLeft = 22;
		var motorRight = 21;
		var servo = -1;
		return new ClimberData(List.of(motorLeft, motorRight, servo));
	}

	@Override
	public void oneStepUp() {
		if (servoLinks.getAngle() <= 0 + Constants.Climber.servoZeroTollerance
				|| servoRechts.getAngle() <= 0 + Constants.Climber.servoZeroTollerance) {
			System.err.println("Cannot move climber if servo isn't retracted");
			return;
		}
		if (seilMotorLinks.getEncoderTicks() > Constants.Climber.ausfahrBereich) {
			System.err.println("Cannot move climber: Already fully released");
			return;
		}
		seilMotorLinks.set(Constants.Climber.manualClimberMovementSpeed);
		seilMotorRechts.set(Constants.Climber.manualClimberMovementSpeed);
	}

	@Override
	public void oneStepDown() {
		seilMotorLinks.set(-Constants.Climber.manualClimberMovementSpeed);
		seilMotorRechts.set(-Constants.Climber.manualClimberMovementSpeed);
	}

	@Override
	public Servo getServo() {
		return servoLinks;
	}
}
