package frc.robot.subsystems;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.fridowpi.command.FridoCommand;
import frc.fridowpi.command.SequentialCommandGroup;
import frc.fridowpi.motors.FridoCanSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.FridolinsMotor.PidType;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BShooter;

public class ShooterSubsystem extends BShooter {
	private final double maxSpeedRpm = 5000.0;

	private FridolinsMotor motorLeft;
	private FridolinsMotor motorRight;
	private FridolinsMotor motorFeeder;
	private FridolinsMotor motorBrushes;

	public ShooterSubsystem shooter;
	private double speedRpm = 5000.0;
	private double targetSpeedRpm = 0;
	private boolean enabled = true;
	private double maxVolts = 12.0;
	public List<Double> speedsMapShooter;
	public List<Double> speedsMapFeeder;
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

	@Override
	public void init() {
		speedsMapShooter = getData().speeds.subList(0, 3);
		speedsMapFeeder = getData().speeds.subList(3, 6);
		speedsMapBrushes = getData().speeds.subList(6, 9);

		assert speedsMapShooter == List.of(
			Constants.Shooter.shooterIntakeSpeed,
			Constants.Shooter.shooterAmpSpeed,
			Constants.Shooter.shooterSpeakerSpeed);
		assert speedsMapFeeder == List.of(
			Constants.Shooter.feedIntakeSpeed,
			Constants.Shooter.feedAmpSpeed,
			Constants.Shooter.feedSpeakerSpeed);
		assert speedsMapBrushes == List.of(
			Constants.Shooter.brushesIntakeSpeed,
			Constants.Shooter.brushesAmpSpeed,
			Constants.Shooter.brushesSpeakerSpeed);
		System.out.println("assertion passed");
		
		System.out.println("shooter: " + speedsMapShooter);
		System.out.println("feeder: " + speedsMapFeeder);
		System.out.println("brushers: " + speedsMapBrushes);
		motorLeft = new FridoCanSparkMax(getData().motorIds.get(0), MotorType.kBrushless);
		motorRight = new FridoCanSparkMax(getData().motorIds.get(1), MotorType.kBrushless);
		motorFeeder = new FridoCanSparkMax(getData().motorIds.get(2), MotorType.kBrushless);
		motorBrushes = new FridoCanSparkMax(getData().motorIds.get(3), MotorType.kBrushless);

		motorLeft.configEncoder(FridoFeedBackDevice.kBuildin, 1);

		motorLeft.factoryDefault();
		motorRight.factoryDefault();
		motorFeeder.factoryDefault();

		motorLeft.setInverted(true);
		motorRight.follow(motorLeft, DirectionType.invertMaster);

		motorFeeder.setInverted(true);

		motorRight.setPID(new PidValues(0, 0, 0)); // We don't use hardware pids
		motorLeft.setPID(new PidValues(0, 0, 0));

		motorRight.configEncoder(FridoFeedBackDevice.kBuildin, getData().countsPerRevolution);
		motorRight.selectPidSlot(0);
		motorFeeder.configEncoder(FridoFeedBackDevice.kBuildin, getData().countsPerRevolution);
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
			// motorLeft.stopMotor();
			return;
		}
		var output = recalculateMotorOutput();
		var actualOutput = output / maxVolts;
		// motorLeft.setVelocity(actualOutput);
	}

	private double recalculateMotorOutput() {
		var ffOutput = ff.calculate(speedRpm);
		var pidOutput = pid.calculate(speedRpm, motorLeft.getEncoderVelocity());
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
		targetSpeedRpm = speed * maxSpeedRpm;
	}

	@Override
	public double getSpeedPercent() {
		return speedRpm / maxSpeedRpm;
	}

	@Override
	public void shoot(IShooterConfig configuration) {
		if (configuration.asInt() == ShooterConfig.INTAKE.asInt()) {
			new IntakeCommand().schedule();
		} else if (configuration.asInt() == ShooterConfig.AMP.asInt()) {
			new ShootAmp().schedule();
		} else if (configuration.asInt() == ShooterConfig.SPEAKER.asInt()) {
			new ShootSpeaker().schedule();
		}
	}

	@Override
	public void stopMotors() {
		motorBrushes.stopMotor();
		motorFeeder.stopMotor();
		motorLeft.stopMotor();
		motorRight.stopMotor();
	}

	public class IntakeCommand extends SequentialCommandGroup {
		private BShooter shooter;

		public IntakeCommand() {
			shooter = Config.active.getShooter().get();
			var config = ShooterConfig.INTAKE.asInt();
			addRequirements(shooter);
			addCommands(
					new InstantCommand(() -> motorBrushes.set(speedsMapBrushes.get(config))),
					new InstantCommand(() -> motorLeft.set(speedsMapShooter.get(config))),
					new InstantCommand(() -> motorFeeder.set(speedsMapFeeder.get(config)))
			);
		}
	}

	public class ShootAmp extends SequentialCommandGroup {
		private BShooter shooter;

		public ShootAmp() {
			shooter = Config.active.getShooter().get();
			var config = ShooterConfig.AMP;
			addRequirements(shooter);
			addCommands(
					new SetBrushMotor(config),
					new SetShooterMotors(config),
					new WaitCommand(1.0),
					new SetFeederMotor(config),
					new WaitCommand(3.0),
					new InstantCommand(shooter::stopMotors));
		}
	}

	public class ShootSpeaker extends ParallelRaceGroup {
		private BShooter shooter;

		public ShootSpeaker() {
			shooter = Config.active.getShooter().get();
			var config = ShooterConfig.SPEAKER;
			addRequirements(shooter);
			addCommands(
					new SequentialCommandGroup(
							new SetBrushMotor(config),
							new Command() {
								@Override
								public boolean isFinished() {
									return false;
								}
							}),
					// Shooter
					new Command() {
						@Override
						public void initialize() {
							setSpeedPercent(speedsMapShooter.get(config.asInt()));
						}

						@Override
						public void execute() {
							speedRpm = motorLeft.getEncoderVelocity();
							var pidOut = pid.calculate(speedRpm, targetSpeedRpm);
							var ffOut = ff.calculate(targetSpeedRpm);
							motorLeft.setVoltage(pidOut + ffOut);
						}

						@Override
						public void end(boolean interrupted) {
							motorLeft.stopMotor();
						}

						@Override
						public boolean isFinished() {
							return false;
						}
					},
					// Feeder
					new SequentialCommandGroup(
							new WaitCommand(1.0),

							new InstantCommand(() -> System.out.println("feeder set")),
							new InstantCommand(
									() -> motorFeeder.set(speedsMapFeeder.get(config.asInt()))),
							new WaitCommand(1.5),
							new InstantCommand(shooter::stopMotors)));
		}
	}

	// Helper Commands
	public class SetBrushMotor extends FridoCommand {
		private double targetSpeed;

		public SetBrushMotor(double speed) {
			targetSpeed = speed;
		}

		public SetBrushMotor(ShooterConfig config) {
			targetSpeed = speedsMapBrushes.get(config.asInt());
		}

		@Override
		public void initialize() {
			motorBrushes.set(targetSpeed);
		}

		@Override
		public boolean isFinished() {
			// return motorLeft.pidAtTarget();
			return true;
		}
	}

	public class SetShooterMotors extends FridoCommand {
		private double targetSpeed;

		public SetShooterMotors(double speed) {
			targetSpeed = speed;
		}

		public SetShooterMotors(ShooterConfig config) {
			targetSpeed = speedsMapShooter.get(config.asInt());
		}

		@Override
		public void initialize() {
			// shooter.setSpeedPercent(targetSpeed);
			motorLeft.set(targetSpeed);
		}

		@Override
		public boolean isFinished() {
			// return motorLeft.pidAtTarget();
			return true;
		}
	}

	public class SetFeederMotor extends FridoCommand {
		private double targetSpeed;

		public SetFeederMotor(double speed) {
			targetSpeed = speed;
		}

		public SetFeederMotor(ShooterConfig config) {
			targetSpeed = speedsMapFeeder.get(config.asInt());
		}

		@Override
		public void initialize() {
			// motorFeeder.setPidTarget(targetSpeed * maxSpeedRpm, PidType.velocity);
			motorFeeder.set(targetSpeed);
		}

		@Override
		public void execute() {
			// motorFeeder.setVelocity(targetSpeed);
		}

		@Override
		public boolean isFinished() {
			// return motorFeeder.pidAtTarget();
			return true;
		}
	}

	@Override
	public void enable() {
		enabled = true;
	}

	@Override
	public void disable() {
		stopMotors();
		enabled = false;
	}

	@Override
	public ShooterData getData() {
		return Constants.Shooter.data;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("ampShooterSpeed",
				() -> speedsMapShooter.get(ShooterConfig.AMP.asInt()),
				val -> speedsMapShooter.set(ShooterConfig.AMP.asInt(), val));
		builder.addDoubleProperty("ampBrushesSpeed",
				() -> speedsMapBrushes.get(ShooterConfig.AMP.asInt()),
				val -> speedsMapBrushes.set(ShooterConfig.AMP.asInt(), val));
		builder.addDoubleProperty("ampFeederSpeed",
				() -> speedsMapShooter.get(ShooterConfig.AMP.asInt()),
				val -> speedsMapShooter.set(ShooterConfig.AMP.asInt(), val));
	}
}
