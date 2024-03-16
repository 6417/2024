package frc.robot.subsystems;

import static java.lang.Math.abs;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.fridolib.QuickCmd;
import frc.fridowpi.command.FridoCommand;
import frc.fridowpi.command.SequentialCommandGroup;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.motors.FridoCanSparkMax;
import frc.fridowpi.motors.FridoServoMotor;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.FridolinsMotor.PidType;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BClimber;
import frc.robot.commands.drive.commands_2024.DriveCommand2024;
import frc.robot.commands.tankdrive.DriveCommand;
import frc.robot.joystick.IdsWithState;
import frc.robot.joystick.Joystick2024;
import frc.robot.joystick.IdsWithState.State;

public class ClimberSubsystem extends BClimber {
	private FridolinsMotor seilMotorLinks = new FridoCanSparkMax(Constants.Climber.seilZiehMotorLinks,
			MotorType.kBrushless);
	private FridolinsMotor seilMotorRechts = new FridoCanSparkMax(Constants.Climber.seilZiehMotorRechts,
			MotorType.kBrushless);
	private FridoServoMotor servoLinks = new FridoServoMotor(Constants.Climber.servoLinksId);
	private FridoServoMotor servoRechts = new FridoServoMotor(Constants.Climber.servoRechtsId);

	/** Creates a new ClimberSubsystem. */
	public ClimberSubsystem() {
	}

	@Override
	public void init() {
		seilMotorLinks.factoryDefault();
		seilMotorRechts.factoryDefault();

		seilMotorLinks.setIdleMode(IdleMode.kCoast);
		seilMotorRechts.setIdleMode(IdleMode.kCoast);

		seilMotorLinks.setPID(Constants.Climber.pidValuesSlot0);
		seilMotorRechts.setPID(Constants.Climber.pidValuesSlot0);

		seilMotorLinks.configEncoder(FridoFeedBackDevice.kBuildin, 1);
		seilMotorRechts.configEncoder(FridoFeedBackDevice.kBuildin, 1);

		seilMotorLinks.setEncoderPosition(0);
		seilMotorRechts.setEncoderPosition(0);

		((CANSparkMax) seilMotorLinks).enableSoftLimit(SoftLimitDirection.kForward, true);
		((CANSparkMax) seilMotorRechts).enableSoftLimit(SoftLimitDirection.kForward, true);

		((CANSparkMax) seilMotorLinks).setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Climber.maxExtentionEncoderTicks);
		((CANSparkMax) seilMotorRechts).setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Climber.maxExtentionEncoderTicks);

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

	double joystickDeadBand = 0.15;
	boolean hasJoystickJustBeenUsed = false;

	@Override
	public void periodic() {
		var y = JoystickHandler.getInstance().getJoystick(Constants.Joystick.secondaryJoystickId)
				.getY();
		if (Math.abs(y) > joystickDeadBand) {
			setSpeed(DriveCommand2024.applyDeadband(y, joystickDeadBand));
			hasJoystickJustBeenUsed = true;
		} else if (hasJoystickJustBeenUsed) {
			hasJoystickJustBeenUsed = false;
			stopMotors();
		}
	}

	@Override
	public void release() {
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
		new ClimberPid(this, Constants.Climber.zielPosition).schedule();
	}

	public class ClimberPid extends FridoCommand {
		private ClimberSubsystem subsystem;
		private double target;

		public ClimberPid(ClimberSubsystem subsystem, double target) {
			if (target > Constants.Climber.maxExtentionEncoderTicks) {
				target = Constants.Climber.maxExtentionEncoderTicks;
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
	public void stopMotors() {
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
		setSpeed(speed + speedAdditon);
		System.out.println(speed);
	}

	private void setSpeed(double speed) {
		if (IdsWithState.activeState != State.ENDGAME) {
			System.err.println("Can't use climber: Not in ENDGAME'");
			return;
		}
		speed = Math.max(speed, Constants.Climber.minimumAusfahrBereich);
		seilMotorLinks.set(speed);
		seilMotorRechts.set(speed);
	}

	@Override
	public FridoServoMotor getServoLeft() {
		return servoLinks;
	}

	@Override
	public FridoServoMotor getServoRight() {
		return servoRechts;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Encoder ticks", seilMotorRechts::getEncoderTicks, null);
	}
}
