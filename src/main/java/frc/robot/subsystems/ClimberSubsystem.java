package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.fridowpi.command.FridoCommand;
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

		seilMotorRechts.setIdleMode(IdleMode.kCoast);
		seilMotorLinks.setIdleMode(IdleMode.kCoast);

		seilMotorLinks.setPID(Constants.Climber.pidValuesSlot0);
		seilMotorRechts.setPID(Constants.Climber.pidValuesSlot0);
		seilMotorLinks.enableForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen, true);
		seilMotorRechts.enableForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen, true);
		seilMotorRechts.follow(seilMotorLinks, DirectionType.followMaster);

		servoRechts.setBoundsMicroseconds(2200, 1499, 1500, 1501, 800);
		servoRechts.setMaxAngle(130);
		servoLinks.setBoundsMicroseconds(2200, 1499, 1500, 1501, 800);
		servoLinks.setMaxAngle(130);

		servoLinks.setAngle(Constants.Climber.servoLeftLockAngle);
		servoRechts.setAngle(Constants.Climber.servoRightLockAngle);
	}

	@Override
	public void run() {
	}

	@Override
	public void release() {
		servoRechts.setAngle(Constants.Climber.servoRightReleaseAngle);
		servoLinks.setAngle(Constants.Climber.servoLeftReleaseAngle);
	}

	@Override
	public void lock() {
		servoRechts.setAngle(Constants.Climber.servoRightLockAngle);
		servoLinks.setAngle(Constants.Climber.servoLeftLockAngle);
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
	public void oneStepUp(double speed) {
		seilMotorLinks.set(-speed);
		seilMotorRechts.set(-speed);
	}

	@Override
	public FridoServoMotor getServoLeft() {
		return servoLinks;
	}

	@Override
	public FridoServoMotor getServoRight() {
		return servoRechts;
	}
}
