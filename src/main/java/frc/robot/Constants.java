package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.fridowpi.motors.FridoTalonSRX;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.LimitSwitchPolarity;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.abstraction.RobotData;
import frc.robot.abstraction.RobotData.AutoData;
import frc.robot.abstraction.RobotData.DriveData;
import frc.robot.abstraction.RobotData.HardwareData;
import frc.robot.abstraction.RobotData.PidData;
import frc.robot.abstraction.baseClasses.BShooter.ShooterData;
import frc.robot.subsystems.drive.swerve.SwerveModule;
import frc.robot.subsystems.drive.tankdrive.MotorSet.MotorRole;

public class Constants {

	public static final boolean driveEnabled = true;

	public static final class Joystick {
        public static final IJoystickId primaryJoystickId = () -> 0;
        public static int idCounterStart = 1000;
	}

	public static final class Global {
		public static final int idShooterMotor = 0;
	}

	public static final class Sysid {
		public static final boolean isTuning = false;
	}

	public static final class Shooter {
		public static final boolean enabled = false;
		public static final double OptimalAmpSpeed = 0.3;
		public static final double OptimalSpeakerSpeed = 0.9;
		public static final double OptimalIntakeSpeed = -0.6;

		public static final ShooterData data = new ShooterData(
				enabled, List.of(-1, -1));
	}

    public static final class Climber {
		public static boolean enabled = false;

        public static final int seilZiehMotorLinks = 1;
        public static final int seilZiehMotorRechts = 2;
        public static final int federLoslassMotorLinks = 2;
        public static final int federLoslassMotorRechts = 4;

        public static final PidValues pidValuesSlot0 = new PidValues(0, 0, 0);
        public static final double toleranzDerHoheDerMotoren = 0.1;
        public static final double runterZiehSpeed = 0.2;
        public static final double ausfahrBereich = 24;
        public static final double zielPosition = 2;

        public static final double manualClimberMovementSpeed = 0.5;
    }

	public static final class Diplodocus {
		public static final boolean driveEnabled = false;

		public static final int mLeftFront = 11;
		public static final int mRightFront = 10;
		public static final int mLeftBack = 13;
		public static final int mRightBack = 12;

		public static final List<MotorRole> invertedMotors = List.of();

		public static final class Odometry {
			public static final Measure<Distance> wheelCircumstance = Meters.of(0.47);
			public static final Measure<Distance> trackWidth = Meters.of(0.5);

			/* Transmission: denotes how many revolution the motor makes compared to the wheel */
			public static final double transmission = 10.71;
			public static final int encoderResolution = 2048;

			public static final double encoderToMeters = wheelCircumstance.in(Meters) / (transmission * encoderResolution);
		}

		public static final class Autonomous {

			/* Values determined through testing */
			public static final Measure<Velocity<Distance>> maxSpeed = MetersPerSecond.of(3.3);
			public static final Measure<Velocity<Velocity<Distance>>> maxAcceleration = MetersPerSecondPerSecond.of(1.2);
			public static final Measure<Velocity<Velocity<Distance>>> maxCentripetalAcceleration = MetersPerSecondPerSecond.of(1.2);

			/* Values determined through SysId tuning */
			public static final Measure<Distance> kS = Meters.of(0.14604);
			public static final Measure<Velocity<Distance>> kV = MetersPerSecond.of(2.3639);
			public static final Measure<Velocity<Velocity<Distance>>> kA = MetersPerSecondPerSecond.of(0.35094);

			public static final Measure<Distance> ramseteB = Meters.of(2.0);
			public static final Measure<Time> ramseteZeta = Seconds.of(0.7);

			public static final PidValues pathWeaverPid = new PidValues(0.36205, 0.0, 0.0);
			public static final PidValues drivePid = new PidValues(0.01, 0.0, 0.0);
		}

		public static final RobotData robotData = new RobotData(
				new HardwareData(
					Odometry.wheelCircumstance,
						Odometry.trackWidth,
						Odometry.encoderToMeters),
				new DriveData(
					driveEnabled,
					List.of(mLeftFront, mRightFront, mLeftBack, mRightBack),
					invertedMotors),
				new AutoData(
					Autonomous.maxSpeed,
					Autonomous.maxAcceleration,
					Autonomous.kS,
					Autonomous.kV,
					Autonomous.kA,
					Autonomous.ramseteB,
					Autonomous.ramseteZeta),
				new PidData(
					Autonomous.pathWeaverPid,
					Autonomous.drivePid,
					Autonomous.drivePid.clone()));
	}

	public static final class Testchassis {
		public static final boolean driveEnabled = true;

		final public static Integer idLeftfront = 12;
		final public static int idRigthfront = 13;
		final public static Integer idLeftback = 0;
		final public static Integer idRigthback = 11;

		public static final List<MotorRole> invertedMotors = List.of(MotorRole.LeftMaster);

		public static final class Odometry {
			public static final double wheelPerimeterMeters = 0.47;
			public static final double trackWidthMeters = 0.5;
			// The transmission denotes how many revolution the motor makes compared to the
			// wheel
			public static final double transmission = 10.71;
			public static final int encoderResolution = 2048;

			public static final double encoderToMetersConversion = 1
					/ ((1 / wheelPerimeterMeters) * transmission * encoderResolution);
		}

		public static final class PathWeaver {
			public static final double ksMeters = 0.12091;
			public static final double kvMetersPerSecoond = 2.3501;
			public static final double ka = 0.21997;

			public static final double kMaxVMetersPerSecond = 1;
			public static final double kMaxAccMetersPerSecond = 0.9;
			public static final double kMaxCentripetalAcceleration = 0;

			public static final double kRamsetB = 2.0;
			public static final double kRamseteZeta = 0.7;

			public static final double kP = 0.36205;
			public static final double kI = 0;
			public static final double kD = 0;

		}

		final public static double ksVolts = 0.40; // random value
		final public static double kvVoltSevondsPerMeter = 1.98; // random value
		final public static double kaVoltSecondsSquaredPerMeter = 0.4; // random value
		final public static double kPDriveVel = 0.1; // random value

		final public static double ticsToMeter = 0.046;

		public static final RobotData robotData = new RobotData(
				new HardwareData(
						Meters.of(Odometry.wheelPerimeterMeters),
						Meters.of(Odometry.trackWidthMeters),
						Odometry.encoderToMetersConversion),
				new DriveData(
					driveEnabled,
					List.of(idLeftfront, idRigthfront, idLeftback, idRigthback),
					invertedMotors),
				new AutoData(
						MetersPerSecond.of(PathWeaver.kMaxVMetersPerSecond),
						MetersPerSecondPerSecond.of(PathWeaver.kMaxAccMetersPerSecond),
						Meters.of(PathWeaver.ksMeters),
						MetersPerSecond.of(PathWeaver.kvMetersPerSecoond),
						MetersPerSecondPerSecond.of(PathWeaver.ka),
						Meters.of(PathWeaver.kRamsetB),
						Seconds.of(PathWeaver.kRamseteZeta)),
				new PidData(
						new PidValues(PathWeaver.kP, PathWeaver.kI, PathWeaver.kD),
						new PidValues(kPDriveVel, 0.0, 0.0),
						new PidValues(kPDriveVel, 0.0, 0.0)));
	}

	public static final class SwerveDrive {

		public static enum MountingLocations {
			FrontRight, FrontLeft, BackRight, BackLeft
		}

		public static final class ButtounIds {
			public static final IJoystickButtonId zeroEncoders = Logitech.back;
			public static final IJoystickButtonId fullSpeed = Logitech.x;
			public static final IJoystickButtonId slowSpeed = Logitech.y;
			public static IJoystickButtonId fieldOriented = Logitech.b;
			public static IJoystickButtonId shooterFrontOriented = Logitech.a;
			public static IJoystickButtonId shooterBackOriented = Logitech.a;
			public static IJoystickButtonId zeroNavx = Logitech.start;
		}

		private static void setSwerveDriveConstants() {
			zeroingSpeed = 500;
			maxFineTuneOffsetForZeroEncodersCommand = 196608 / 100;
			maxSpeedOfDrive = 1;
		}

		public static final double gearRatio = 1.0 / 5.192308;

		public static final boolean enabled = false;
		public static final boolean rotateAllModulesInSameDirection = false;
		public static final boolean joystickYinverted = true;
		public static final boolean joystickXinverted = true;
		public static double zeroingSpeed;
		public static final double deadBand = 0.015;
		public static final double yOffsetMapperMaxVoltage = 12.5;
		public static final double yOffsetMapperMinVoltage = 9;
		public static final double finetuningZeroFactor = 0.1;
		public static double maxFineTuneOffsetForZeroEncodersCommand;
		public static double maxSpeedOfDrive; // in meters per second
		public static final double maxRotationSpeed = 15 * Math.PI / 16; // at full rotation speed the robot will turn
																			// by 180 degrees, in rad per second
		public static final Map<MountingLocations, SwerveModule.Config> swerveModuleConfigs = new HashMap<>();

		public static SwerveModule.Config commonConfigurations = new SwerveModule.Config();
		public static double defaultSpeedFactor = 0.75;
		public static double slowSpeedFactor = 0.35;
		public static double fullSpeed = 1.0;

		static {
			setSwerveDriveConstants();
			addCommonModuleConfigurarions();
			addModuleSpecificConfigurarions();
		}

		private static void addCommonModuleConfigurarions() {
			commonConfigurations.driveMotorTicksPerRotation = 11_564.0;
			commonConfigurations.rotationMotorTicksPerRotation = 196_608.0;
			commonConfigurations.drivePID = new PidValues(0.015, 0.0, 0.0, 0.03375);
			commonConfigurations.drivePID.slotIdX = Optional.of(0);
			// commonConfigurations.drivePID.setAcceleration(0.0000001);
			commonConfigurations.rotationPID = new PidValues(0.04, 0.0, 0.5);
			commonConfigurations.rotationPID.slotIdX = Optional.of(0);
			commonConfigurations.wheelCircumference = 0.1 * Math.PI;
			commonConfigurations.maxVelocity = maxSpeedOfDrive;
			commonConfigurations.driveEncoderType = FridoFeedBackDevice.kRelative;
			commonConfigurations.rotationEncoderType = FridoFeedBackDevice.kRelative;
			commonConfigurations.limitSwitchPolarity = LimitSwitchPolarity.kNormallyOpen;
			// commonConfigurations.driveAccelerationForward = 2000;
			// commonConfigurations.driveAccelerationSideWays = 500;
			// commonConfigurations.problemDirectionsWhileBreaking = new Vector2[] {
			// new Vector2(-1 / Math.sqrt(2), -1 / Math.sqrt(2)),
			// new Vector2(-1 / Math.sqrt(2), 1 / Math.sqrt(2)) };
			// commonConfigurations.problemDirectionsBreakModeGauseStrechingFactor = 1.0;
		}

		private static FridoTalonSRX angleMotorInitializer(int id, MotorType motorType) {
			FridoTalonSRX motor = new FridoTalonSRX(id);
			motor.factoryDefault();
			// motor.enableVoltageCompensation(10.4);
			return motor;
		}

		private static FridolinsMotor driveMotorInitializer(int id, MotorType motorType) {
			FridoTalonSRX motor = angleMotorInitializer(id, motorType);
			motor.enableForwardLimitSwitch(LimitSwitchPolarity.kDisabled, false);
			motor.enableReverseLimitSwitch(LimitSwitchPolarity.kDisabled, false);
			// motor.enableSoftLimit(SoftLimitDirection.kForward, false);
			// motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
			return motor;
		}

		private static void addModuleSpecificConfigurarions() {
			SwerveModule.Config frontLeftConfig = commonConfigurations.clone();
			frontLeftConfig.mountingPoint = new Translation2d(0.32, 0.305);
			frontLeftConfig.driveMotorInitializer = () -> driveMotorInitializer(32, MotorType.kBrushless);
			frontLeftConfig.rotationMotorInitializer = () -> angleMotorInitializer(33, MotorType.kBrushless);
			frontLeftConfig.driveMotorInverted = true;
			frontLeftConfig.halSensorPosition = -187_149.0 - 10_431;
			swerveModuleConfigs.put(MountingLocations.FrontLeft, frontLeftConfig);

			SwerveModule.Config frontRightConfig = commonConfigurations.clone();
			frontRightConfig.mountingPoint = new Translation2d(-0.32, 0.305);
			frontRightConfig.driveMotorInitializer = () -> driveMotorInitializer(38, MotorType.kBrushless);
			frontRightConfig.rotationMotorInitializer = () -> angleMotorInitializer(39, MotorType.kBrushless);
			frontRightConfig.driveMotorInverted = false;
			frontRightConfig.halSensorPosition = 195_605.0;
			swerveModuleConfigs.put(MountingLocations.FrontRight, frontRightConfig);

			SwerveModule.Config backLeftConfig = commonConfigurations.clone();
			backLeftConfig.mountingPoint = new Translation2d(0.32, -0.305);
			backLeftConfig.driveMotorInitializer = () -> driveMotorInitializer(34, MotorType.kBrushless);
			backLeftConfig.rotationMotorInitializer = () -> angleMotorInitializer(35, MotorType.kBrushless);
			backLeftConfig.driveMotorInverted = true;
			backLeftConfig.halSensorPosition = 187_701.0;
			swerveModuleConfigs.put(MountingLocations.BackLeft, backLeftConfig);

			SwerveModule.Config backRightConfig = commonConfigurations.clone();
			backRightConfig.mountingPoint = new Translation2d(-0.32, -0.305);
			backRightConfig.driveMotorInitializer = () -> driveMotorInitializer(36, MotorType.kBrushless);
			backRightConfig.rotationMotorInitializer = () -> angleMotorInitializer(37, MotorType.kBrushless);
			backRightConfig.driveMotorInverted = false;
			backRightConfig.halSensorPosition = 187_420.0;
			swerveModuleConfigs.put(MountingLocations.BackRight, backRightConfig);
		}

	}
}
