package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.fridowpi.command.FridoCommand;
import frc.fridowpi.command.SequentialCommandGroup;
import frc.fridowpi.motors.FridoCanSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.FridolinsMotor.PidType;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BShooter;

public class ShooterSubsystem extends BShooter {
	private final double maxSpeedRpm = 5000.0;

	private final FridolinsMotor motorLeft = new FridoCanSparkMax(getData().motorIds.get(0), MotorType.kBrushless);
	private final FridolinsMotor motorRight = new FridoCanSparkMax(getData().motorIds.get(1), MotorType.kBrushless);
	private final FridolinsMotor motorFeeder = new FridoCanSparkMax(getData().motorIds.get(2), MotorType.kBrushless);
	private final FridolinsMotor motorBrushes = new FridoCanSparkMax(getData().motorIds.get(3), MotorType.kBrushless);
	public ShooterSubsystem shooter;
	private double speedRpm = 5000.0;
	private boolean enabled = true;
	private double maxVolts = 12.0;
	public List<Double> speedsMapFeeder;
	public List<Double> speedsMapShooter;
	public List<Double> speedsMapBrushes;

	private final PIDController pid = new PIDController(0.0005, 0.0, 0.0);
	private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.081, 0.0020075);

	/**
	 * Left follows right
	 * Right is inverted
	 * Slot Right-0 and Feeder-0 are used for velocity control
	 **/

	public static enum ShooterConfig implements IShooterConfig {
		INTAKE(0),
		AMP(1),
		SPEAKER(2);

		private final int id;

		private ShooterConfig(int id) {
			this.id = id;
		}

		@Override
		public int asInt() {
			return id;
		}
	}

	public ShooterSubsystem() {
		shooter = this;
	}

	public void init() {
		speedsMapShooter = getData().speeds;

		motorLeft.setInverted(true);
		motorRight.follow(motorLeft, DirectionType.invertMaster);

		motorFeeder.setInverted(true);

		motorRight.selectPidSlot(0);
		motorFeeder.selectPidSlot(0);

		motorRight.setIdleMode(IdleMode.kCoast);
		motorLeft.setIdleMode(IdleMode.kCoast);
		motorFeeder.setIdleMode(IdleMode.kCoast);

		((CANSparkMax) motorRight).setSmartCurrentLimit(20);
		((CANSparkMax) motorLeft).setSmartCurrentLimit(20);
		((CANSparkMax) motorFeeder).setSmartCurrentLimit(40);

		((CANSparkMax) motorRight).enableVoltageCompensation(12);
		((CANSparkMax) motorLeft).enableVoltageCompensation(12);
		((CANSparkMax) motorFeeder).enableVoltageCompensation(12);

		pid.setTolerance(50);
		pid.setIntegratorRange(-0.015, 0.015);
	}

	@Override
	public void run() {
		if (!enabled) {
			motorRight.stopMotor();
			return;
		}
		var output = recalculateMotorOutput();
		var actualOutput = output / maxVolts;
		motorRight.setVelocity(actualOutput);
	}

	private double recalculateMotorOutput() {
		var ffOutput = ff.calculate(speedRpm);
		var pidOutput = pid.calculate(speedRpm, motorRight.getEncoderVelocity());
		return Math.max(-0.5, pidOutput + ffOutput); // Necessary? set(0) is fine?
	}

	@Override
	public void setSpeedPercent(double speed) {
		if (speed > 1.0) {
			System.out.println("<Warning> Speed too high: " + speed);
			speed = 1.0;
		} else if (speed < -1.0) {
			System.out.println("<Warning> Speed too low: " + speed);
			speed = -1.0;
		}
		speedRpm = speed * maxSpeedRpm;
	}

	@Override
	public double getSpeedPercent() {
		return speedRpm / maxSpeedRpm;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("shooterSpeeds", () -> speedRpm, val -> speedRpm = val);
	}

	@Override
	public void shoot(IShooterConfig configuration) {
		if (configuration.asInt() == ShooterConfig.INTAKE.asInt()) {
			CommandScheduler.getInstance().schedule(new IntakeCommand());
		} else if (configuration.asInt() == ShooterConfig.AMP.asInt()) {
			CommandScheduler.getInstance().schedule(new ShootAmp());
		} else if (configuration.asInt() == ShooterConfig.SPEAKER.asInt()) {
			CommandScheduler.getInstance().schedule(new ShootSpeaker());
		}
	}

	public class IntakeCommand extends SequentialCommandGroup {
		private BShooter shooter;

		public IntakeCommand() {
			shooter = Config.active.getShooter().get();
			addRequirements(shooter);
			addCommands(
					new SetBrushMotor(ShooterConfig.INTAKE),
					new SetShooterMotors(ShooterConfig.INTAKE),
					new SetFeederMotor(ShooterConfig.INTAKE));
		}
	}

	public class ShootSpeaker extends SequentialCommandGroup {
		private BShooter shooter;

		public ShootSpeaker() {
			shooter = Config.active.getShooter().get();
			addRequirements(shooter);
			addCommands(
					new SetBrushMotor(ShooterConfig.SPEAKER),
					new SetShooterMotors(ShooterConfig.SPEAKER),
					new SetFeederMotor(ShooterConfig.SPEAKER),
					new WaitCommand(3.0),
					new SetFeederMotor(0),
					new SetShooterMotors(0),
					new SetBrushMotor(0));
		}
	}

	public class ShootAmp extends SequentialCommandGroup {
		private BShooter shooter;

		public ShootAmp() {
			shooter = Config.active.getShooter().get();
			addRequirements(shooter);
			addCommands(
					new SetBrushMotor(ShooterConfig.AMP),
					new SetShooterMotors(ShooterConfig.AMP),
					new SetFeederMotor(ShooterConfig.AMP),
					new WaitCommand(3.0),
					new SetFeederMotor(0),
					new SetShooterMotors(0),
					new SetBrushMotor(0));
		}
	}

	// Helper Commands
	public class SetBrushMotor extends FridoCommand {
		private double targetSpeed;

		public SetBrushMotor(double speed) {
			targetSpeed = speed;
		}

		public SetBrushMotor(ShooterConfig speed) {
			targetSpeed = speedsMapBrushes.get(speed.asInt());
		}

		@Override
		public void initialize() {
			motorBrushes.set(targetSpeed);
		}

		@Override
		public boolean isFinished() {
			return motorRight.pidAtTarget();
		}
	}

	public class SetShooterMotors extends FridoCommand {
		private double targetSpeed;

		public SetShooterMotors(double speed) {
			targetSpeed = speed;
		}

		public SetShooterMotors(ShooterConfig speed) {
			targetSpeed = speedsMapShooter.get(speed.asInt());
		}

		@Override
		public void initialize() {
			shooter.setSpeedPercent(targetSpeed);
		}

		@Override
		public boolean isFinished() {
			return motorRight.pidAtTarget();
		}
	}

	public class SetFeederMotor extends FridoCommand {
		private double targetSpeed;

		public SetFeederMotor(double speed) {
			targetSpeed = speed;
		}

		public SetFeederMotor(ShooterConfig speed) {
			targetSpeed = speedsMapFeeder.get(speed.asInt());
		}

		@Override
		public void initialize() {
			motorFeeder.setPidTarget(targetSpeed * maxSpeedRpm, PidType.velocity);
		}

		@Override
		public void execute() {
			motorFeeder.setVelocity(targetSpeed);
		}

		@Override
		public boolean isFinished() {
			return motorFeeder.pidAtTarget();
		}
	}

	@Override
	public void enable() {
		enabled = true;
	}

	@Override
	public void disable() {
		enabled = false;
	}

	@Override
	public ShooterData getData() {
		return Constants.Shooter.data;
	}
}
