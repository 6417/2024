package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.fridowpi.command.FridoCommand;
import frc.fridowpi.command.SequentialCommandGroup;
import frc.fridowpi.motors.FridoTalonSRX;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.PidType;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BShooter;

public class ShooterSubsystem extends BShooter {

	private final FridolinsMotor motor_left = new FridoTalonSRX(getData().motorIds.get(0));
	private final FridolinsMotor motor_right = new FridoTalonSRX(getData().motorIds.get(1));
	private final FridolinsMotor motor_feeder = new FridoTalonSRX(getData().motorIds.get(2));
	public ShooterSubsystem shooter;
	private double speeds = 0.0;
	private boolean enabled = true;
	public List<Double> speedsMapFeeder;
	public List<Double> speedsMapShooter;


	/**
	 * Left follows right
	 * Right is inverted
	 **/

	public static final int SHOOTER_CONFIG_INTAKE = 0;
	public static final int SHOOTER_CONFIG_AMP = 1;
	public static final int SHOOTER_CONFIG_SPEAKER = 2;

	public static enum ShooterConfig {
		INTAKE(0),
		AMP(1),
		SPEAKER(2);

		private final int id;

		private ShooterConfig(int id) {
			this.id = id;
		}

		public int asInt() {
			return id;
		}
	}

	public ShooterSubsystem() {
		shooter = this;
	}

	public void init() {
		motor_left.follow(motor_right, DirectionType.invertMaster);

		motor_right.selectPidSlot(0);
		motor_feeder.selectPidSlot(0);
	}

	@Override
	public void run() {
		if (enabled) {
			motor_right.set(speeds);
		} else {
			motor_right.stopMotor();
		}
	}

	@Override
	public void setSpeedPercent(double speed) {
		if (speed > 1.0) {
			System.out.println("Speed too high: " + speed);
			speed = 1.0;
		} else if (speed < -1.0) {
			System.out.println("Speed too low: " + speed);
			speed = -1.0;
		}
		speeds = speed;
	}

	@Override
	public double getSpeedPercent() {
		return speeds;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("shooterSpeeds", () -> speeds, val -> speeds = val);
	}

	@Override
	public void shoot(int configuration) {
		switch (configuration) {
			case SHOOTER_CONFIG_INTAKE:
				CommandScheduler.getInstance().schedule(new IntakeCommand());
				break;
			case SHOOTER_CONFIG_AMP:
				CommandScheduler.getInstance().schedule(new ShootAmp());
				break;
			case SHOOTER_CONFIG_SPEAKER:
				CommandScheduler.getInstance().schedule(new ShootSpeaker());
				break;
			default:
				throw new Error("Unknown configuration: " + configuration);
		}
	}

	public class IntakeCommand extends SequentialCommandGroup {
		private BShooter shooter;

		public IntakeCommand() {
			shooter = Config.active.getShooter().get();
			addRequirements(shooter);
			addCommands(
					new GoToSpeed(ShooterConfig.INTAKE),
					new Feed(ShooterConfig.INTAKE));
		}
	}

	public class ShootSpeaker extends SequentialCommandGroup {
		private BShooter shooter;

		public ShootSpeaker() {
			shooter = Config.active.getShooter().get();
			addRequirements(shooter);
			addCommands(
					new GoToSpeed(ShooterConfig.SPEAKER),
					new Feed(ShooterConfig.SPEAKER),
					new WaitCommand(1.0),
					new Feed(0),
					new GoToSpeed(0));
		}
	}

	public class ShootAmp extends SequentialCommandGroup {
		private BShooter shooter;

		public ShootAmp() {
			shooter = Config.active.getShooter().get();
			addRequirements(shooter);
			addCommands(
					new GoToSpeed(ShooterConfig.AMP),
					new Feed(ShooterConfig.AMP),
					new WaitCommand(1.0),
					new Feed(0),
					new GoToSpeed(0));
		}
	}

	// Helper Commands
	public class GoToSpeed extends FridoCommand {
		private double targetSpeed;

		public GoToSpeed(double speed) {
			targetSpeed = speed;
		}

		public GoToSpeed(ShooterConfig speed) {
			targetSpeed = speedsMapShooter.get(speed.asInt());
		}

		@Override
		public void initialize() {
			motor_right.setPidTarget(targetSpeed, PidType.velocity);
		}

		@Override
		public boolean isFinished() {
			return motor_right.pidAtTarget();
		}
	}

	public class Feed extends FridoCommand {
		private double targetSpeed;

		public Feed(double speed) {
			targetSpeed = speed;
		}

		public Feed(ShooterConfig speed) {
			targetSpeed = speedsMapFeeder.get(speed.asInt());
		}

		@Override
		public void initialize() {
			motor_feeder.setPidTarget(targetSpeed, PidType.velocity);
		}

		@Override
		public boolean isFinished() {
			return motor_feeder.pidAtTarget();
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

