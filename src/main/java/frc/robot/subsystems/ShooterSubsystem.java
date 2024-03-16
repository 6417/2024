package frc.robot.subsystems;

import static frc.robot.Utils.log;

import java.util.List;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.fridowpi.command.FridoCommand;
import frc.fridowpi.command.ParallelCommandGroup;
import frc.fridowpi.command.SequentialCommandGroup;
import frc.fridowpi.motors.FridoCanSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BShooter;

public class ShooterSubsystem extends BShooter {
	private final double maxSpeedRpm = 5000.0;

	private FridolinsMotor motorLeft;
	private FridolinsMotor motorRight;
	private FridolinsMotor motorFeeder;

	private double speedRpm = 5000.0;
	private double shooterTargetSpeedRpm = 0;
	private double feederTargetSpeedRpm = 0;
	public List<Double> speedsMapShooter;
	public List<Double> speedsMapFeeder;
	public List<Double> speedsMapBrushes;

	private final PIDController pid = new PIDController(0.0005, 0.0001, 0.0);
	private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.081, 0.0020075);

	// private PidValues pidValues = new PidValues(0.0005, 0.0, 0.0);
	// private FeedForwardValues ffValues = new FeedForwardValues(0.0081,
	// 0.0020075);

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

		motorLeft.configEncoder(FridoFeedBackDevice.kBuildin, 1);

		motorLeft.factoryDefault();
		motorRight.factoryDefault();
		motorFeeder.factoryDefault();

		motorLeft.setInverted(true);
		motorRight.follow(motorLeft, DirectionType.invertMaster);

		motorFeeder.setInverted(true);

		motorRight.configEncoder(FridoFeedBackDevice.kBuildin, getData().countsPerRevolution);
		motorRight.selectPidSlot(0);
		motorFeeder.configEncoder(FridoFeedBackDevice.kBuildin, getData().countsPerRevolution);
		motorFeeder.selectPidSlot(0);

		pid.setTolerance(30);
		pid.setIntegratorRange(-0.015, 0.015);

		motorRight.setIdleMode(IdleMode.kCoast);
		motorLeft.setIdleMode(IdleMode.kCoast);
		motorFeeder.setIdleMode(IdleMode.kCoast);

		((CANSparkMax) motorRight).setSmartCurrentLimit(20);
		((CANSparkMax) motorLeft).setSmartCurrentLimit(20);
		((CANSparkMax) motorFeeder).setSmartCurrentLimit(40);

		((CANSparkMax) motorRight).enableVoltageCompensation(12);
		((CANSparkMax) motorLeft).enableVoltageCompensation(12);
		((CANSparkMax) motorFeeder).enableVoltageCompensation(12);
	}

	public void setShooterSpeedPercent(double speed) {
		if (speed > 1.0) {
			System.out.println("<Warning> Speed too high: " + speed);
			speed = 1.0;
		} else if (speed < -1.0) {
			System.out.println("<Warning> Speed too low: " + speed);
			speed = -1.0;
		}
		shooterTargetSpeedRpm = speed * maxSpeedRpm;
	}

	public void setFeederSpeedPercent(double speed) {
		if (speed > 1.0) {
			System.out.println("<Warning> Speed too high: " + speed);
			speed = 1.0;
		} else if (speed < -1.0) {
			System.out.println("<Warning> Speed too low: " + speed);
			speed = -1.0;
		}
		feederTargetSpeedRpm = speed * maxSpeedRpm;
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
	public void run() {
	}

	boolean pidShooterEnabled = false;
	boolean pidFeederEnabled = false;

	@Override
	public void stopMotors() {
		pidShooterEnabled = false;
		pidFeederEnabled = false;
		motorFeeder.stopMotor();
		motorLeft.stopMotor();
		motorRight.stopMotor();
	}

	private Command setSpeedFeeder(ShooterConfig config) {
		return new InstantCommand(() -> motorFeeder.set(speedsMapFeeder.get(config.asInt())));
	}

	private Command setSpeedShooterMotors(ShooterConfig config) {
		return new InstantCommand(() -> motorLeft.set(speedsMapShooter.get(config.asInt())));
	}

	public class IntakeCommand extends ParallelCommandGroup {
		public IntakeCommand() {
			addRequirements(ShooterSubsystem.this);
			addCommands(
					new SetShooterMotorsPid(ShooterConfig.INTAKE) {
						@Override
						public boolean isFinished() {
							return !pidShooterEnabled;
						}
					},
					setSpeedFeeder(ShooterConfig.INTAKE));
		}
	}

	public class ShootAmp extends ParallelRaceGroup {
		public ShootAmp() {
			var config = ShooterConfig.AMP;
			var shooter = ShooterSubsystem.this;
			addRequirements(shooter);
			addCommands(
					// Set feeder and keep pid
					new SetFeederPid(config) {
						@Override
						public boolean isFinished() {
							return !pidFeederEnabled;
						}
					},
					// Set shooter and keep pid
					new SetShooterMotorsPid(config) {
						@Override
						public boolean isFinished() {
							return !pidShooterEnabled;
						}
					},
					new SequentialCommandGroup(
							new Command() {
								@Override
								public boolean isFinished() {
									return shooterAtTargetSpeed() && pidShooterEnabled;
								}
							},
							setSpeedFeeder(config),
							new WaitCommand(3.0),
							new InstantCommand(shooter::stopMotors)));
		}
	}

	public class ShootSpeaker extends ParallelRaceGroup {
		public ShootSpeaker() {
			var shooter = ShooterSubsystem.this;
			var config = ShooterConfig.SPEAKER;
			addRequirements(shooter);
			addCommands(
					// Set shooter and feeder and keep pid
					new SetShooterMotorsPid(config) {
						@Override
						public boolean isFinished() {
							return !pidShooterEnabled;
						}
					},
					new SequentialCommandGroup(
							// Wait for shooter to get to full speed
							new Command() {
								@Override
								public boolean isFinished() {
									return shooterAtTargetSpeed() && pidShooterEnabled;
								}
							},
							new ParallelRaceGroup(
									new SetFeederPid(config) {
										@Override
										public boolean isFinished() {
											return !pidFeederEnabled;
										}
									},
									new WaitCommand(2)),
							new InstantCommand(shooter::stopMotors)));
		}
	}

	// Helper Commands
	public class SetShooterMotorsPid extends FridoCommand {
		private double targetSpeed;

		public SetShooterMotorsPid(ShooterConfig config) {
			targetSpeed = speedsMapShooter.get(config.asInt());
		}

		@Override
		public void initialize() {
			setShooterSpeedPercent(targetSpeed);
			pidShooterEnabled = true;
		}

		@Override
		public void execute() {
			if (ShooterSubsystem.this.pidShooterEnabled) {
				speedRpm = motorLeft.getEncoderVelocity();
				var pidOut = pid.calculate(speedRpm, shooterTargetSpeedRpm);
				var ffOut = ff.calculate(shooterTargetSpeedRpm);
				motorLeft.setVoltage(pidOut + ffOut);
			}
		}

		@Override
		public void end(boolean interrupted) {
			stopMotors();
			pidShooterEnabled = false;
		}

		@Override
		public boolean isFinished() {
			return shooterAtTargetSpeed();
		}
	}

	public class SetFeederPid extends FridoCommand {
		private double targetSpeed;

		public SetFeederPid(ShooterConfig config) {
			targetSpeed = speedsMapFeeder.get(config.asInt());
		}

		@Override
		public void initialize() {
			setFeederSpeedPercent(targetSpeed);
			pidFeederEnabled = true;
		}

		@Override
		public void execute() {
			if (ShooterSubsystem.this.pidFeederEnabled) {
				speedRpm = motorFeeder.getEncoderVelocity();
				var pidOut = pid.calculate(speedRpm, feederTargetSpeedRpm);
				var ffOut = ff.calculate(feederTargetSpeedRpm);
				motorFeeder.setVoltage(pidOut + ffOut);
			}
		}

		@Override
		public void end(boolean interrupted) {
			stopMotors();
			pidShooterEnabled = false;
		}

		@Override
		public boolean isFinished() {
			return feederAtTargetSpeed();
		}
	}

	@Override
	public void enable() {
	}

	private boolean feederAtTargetSpeed() {
		return Math.abs(shooterTargetSpeedRpm - motorLeft.getEncoderVelocity())
				/ Math.abs(shooterTargetSpeedRpm) < 0.08;
	}

	private boolean shooterAtTargetSpeed() {
		return Math.abs(shooterTargetSpeedRpm - motorLeft.getEncoderVelocity())
				/ Math.abs(shooterTargetSpeedRpm) < 0.08;
	}

	@Override
	public void disable() {
		stopMotors();
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
				() -> speedsMapFeeder.get(ShooterConfig.AMP.asInt()),
				val -> speedsMapFeeder.set(ShooterConfig.AMP.asInt(), val));
		builder.addDoubleProperty("Left Speed", motorLeft::getEncoderVelocity, null);

		builder.addBooleanProperty("Invert shooter motor 20", () -> motorLeft.getInverted(),
				val -> motorLeft.setInverted(motorLeft.getInverted()));
	}
}
