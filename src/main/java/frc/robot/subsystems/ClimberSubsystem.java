package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.fridolib.QuickCmd;
import frc.fridowpi.command.FridoCommand;
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
	private FridolinsMotor seilMotorLinks;
	private FridolinsMotor seilMotorRechts;
	private FridoServoMotor servoLinks;
	private FridoServoMotor servoRechts;

	/** Creates a new ClimberSubsystem. */
	public ClimberSubsystem() {
	}

	@Override
	public void init() {
		seilMotorLinks = new FridoCanSparkMax(
				Constants.Climber.seilZiehMotorLinks, MotorType.kBrushless);
		seilMotorRechts = new FridoCanSparkMax(
				Constants.Climber.seilZiehMotorRechts, MotorType.kBrushless);

		servoLinks = new FridoServoMotor(Constants.Climber.servoLinksId);
		servoRechts = new FridoServoMotor(Constants.Climber.servoRechtsId);

		seilMotorLinks.factoryDefault();
		seilMotorRechts.factoryDefault();

		seilMotorLinks.setIdleMode(IdleMode.kCoast);
		seilMotorRechts.setIdleMode(IdleMode.kCoast);

		seilMotorLinks.setPID(Constants.Climber.pidValuesSlot0);
		seilMotorRechts.setPID(Constants.Climber.pidValuesSlot0);

		// seilMotorLinks.enableForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen,
		// true);
		// seilMotorRechts.enableForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen,
		// true);

		seilMotorLinks.setInverted(true);
		seilMotorRechts.setInverted(false);
		seilMotorRechts.follow(seilMotorLinks, DirectionType.invertMaster);

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
		System.out.println("test erfolgreich");
		new SequentialCommandGroup(
				QuickCmd.withInit(this::releaseServos),
				new WaitCommand(1),
				QuickCmd.withInit(this::lockServos)).schedule();
	}

	public void releaseServos() {
		servoRechts.setAngle(Constants.Climber.servoRightReleaseAngle);
		servoLinks.setAngle(Constants.Climber.servoLeftReleaseAngle);
	}

	public void lockServos() {
		servoRechts.setAngle(Constants.Climber.servoRightLockAngle);
		servoLinks.setAngle(Constants.Climber.servoLeftLockAngle);
	}

	@Override
	public void retract() {
		new ClimberPid(this, Constants.Climber.pidPosClimbedUp).schedule();
	}

	public class ClimberPid extends FridoCommand {
		private ClimberSubsystem subsystem;
		private double target = Constants.Climber.pidPosClimbedUp;

		public ClimberPid(ClimberSubsystem subsystem, double target) {
			// if (target > Constants.Climber.ausfahrBereich) {
			// target = Constants.Climber.ausfahrBereich;
			// System.err.println("<ClimberSubsystem::ClimberPid> ausfahrBereich
			// ueberschritten");
			// }
			// if (target < Constants.Climber.minimumAusfahrBereich) {
			// target = Constants.Climber.minimumAusfahrBereich;
			// System.err.println("<ClimberSubsystem::ClimberPid> minimaler ausfahrBereich
			// ueberschritten");
			// }
			// this.target = target;
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
		speed = 0;
	}

	@Override
	public ClimberData getData() {
		var motorLeft = 22;
		var motorRight = 21;
		var servo = -1;
		return new ClimberData(List.of(motorLeft, motorRight, servo));
	}

	double speed = 0;

	@Override
	public void oneStepUp(double speedAdditon) {
		speed += speedAdditon;
		speed = Math.max(speed, 0);
		System.out.println(speed);
		seilMotorLinks.set(speed);
		seilMotorRechts.set(speed);
	}
}
