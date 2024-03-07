package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.motors.FridoCanSparkMax;
import frc.fridowpi.motors.FridoFalcon500v6;
import frc.fridowpi.motors.FridoTalonSRX;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.FridolinsMotor.LimitSwitchPolarity;
import frc.fridowpi.motors.utils.FeedForwardValues;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.abstraction.RobotData;
import frc.robot.abstraction.RobotData.AutoData;
import frc.robot.abstraction.RobotData.DriveData;
import frc.robot.abstraction.RobotData.HardwareData;
import frc.robot.abstraction.RobotData.PidData;
import frc.robot.abstraction.baseClasses.BDrive.MountingLocations;
import frc.robot.abstraction.baseClasses.BShooter.ShooterData;
import frc.robot.subsystems.drive.swerve_2019.SwerveModule;
import frc.robot.subsystems.drive.tankdrive.MotorSet.MotorRole;

public class Constants {

	public static final boolean driveEnabled = true;

	public static final class Joystick {
		public static final IJoystickId primaryJoystickId = () -> 0;
		public static final int idCounterStart = 1000;
		public static final double lt_rt_reshold = 0.2;
	}

	public static final class Global {
		public static final int neoTicksPerRevolution = 42;
	}

	public static final class Sysid {
		public static final boolean isTuning = false;
	}

	public static final class Shooter {
		public static final double shooterIntakeSpeed = -0.6;
		public static final double feedIntakeSpeed = 0;
		public static final double brushesIntakeSpeed = -0.1;

		public static final double shooterAmpSpeed = 0.12;
		public static final double feedAmpSpeed = 0.12;
		public static final double brushesAmpSpeed = 0.2;

		public static final double shooterSpeakerSpeed = 0.7;
		public static final double feedSpeakerSpeed = 1.0;
		public static final double brushesSpeakerSpeed = 0;

		public static final PidValues velocityPidShooter = new PidValues(0.0005, 0.0, 0.0);
		public static final FeedForwardValues velocityffShooter = new FeedForwardValues(0.081, 0.0020075);

		public static final ShooterData data = new ShooterData(
				List.of(20, 21, 22, 23),
				List.of(shooterIntakeSpeed, shooterAmpSpeed, shooterSpeakerSpeed,
						feedIntakeSpeed, feedAmpSpeed, feedSpeakerSpeed,
						brushesIntakeSpeed, brushesAmpSpeed, brushesSpeakerSpeed),
				1);
	};

	public static final class Climber {

		public static final int seilZiehMotorLinks = 31;
		public static final int seilZiehMotorRechts = 30;
		public static final int servoLinksId = 0;
		public static final int servoRechtsId = 1;

		public static final PidValues pidValuesSlot0 = new PidValues(0, 0, 0);
		public static final double toleranzDerHoheDerMotoren = 0.1;
		public static final double runterZiehSpeed = 0.2;
		public static final double ausfahrBereich = 24;
		public static final double minimumAusfahrBereich = 0;
		public static final double zielPosition = 2;

		public static final double manualClimberMovementSpeed = 0.5;

		public static final double maxServoPos = 130;
		public static final double servoZeroTollerance = 5.0;

		public static final double servoLeftReleaseAngle = 87;
		public static final double servoLeftLockAngle = 107;

		public static final double servoRightReleaseAngle = 53;
		public static final double servoRightLockAngle = 33;
	}

	public static final class Diplodocus {

		public static final int mLeftFront = 11;
		public static final int mRightFront = 10;
		public static final int mLeftBack = 13;
		public static final int mRightBack = 12;

		public static final List<MotorRole> invertedMotors = List.of();

		public static final class Odometry {
			public static final Measure<Distance> wheelCircumstance = Meters.of(0.47);
			public static final Measure<Distance> trackWidth = Meters.of(0.5);

			/*
			 * Transmission: denotes how many revolution the motor makes compared to the
			 * wheel
			 */
			public static final double transmission = 10.71;
			public static final int encoderResolution = 2048;

			public static final double encoderToMeters = wheelCircumstance.in(Meters)
					/ (transmission * encoderResolution);
		}

		public static final class Autonomous {

			/* Values determined through testing */
			public static final Measure<Velocity<Distance>> maxSpeed = MetersPerSecond.of(3.3);
			public static final Measure<Velocity<Velocity<Distance>>> maxAcceleration = MetersPerSecondPerSecond
					.of(1.2);
			public static final Measure<Velocity<Velocity<Distance>>> maxCentripetalAcceleration = MetersPerSecondPerSecond
					.of(1.2);

			/* Values determined through SysId tuning */
			public static final Measure<Distance> kS = Meters.of(0.14604);
			public static final Measure<Velocity<Distance>> kV = MetersPerSecond.of(2.3639);
			public static final Measure<Velocity<Velocity<Distance>>> kA = MetersPerSecondPerSecond.of(0.35094);

			public static final Measure<Distance> ramseteB = Meters.of(2.0);
			public static final Measure<Time> ramseteZeta = Seconds.of(0.7);

			public static final PidValues pathWeaverPid = new PidValues(0.36205, 0.0, 0.0);
			public static final PidValues drivePid = new PidValues(0.1, 0.0, 0.0);
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

		final public static int idLeftfront = 12;
		final public static int idRightfront = 13;
		final public static int idLeftback = 0;
		final public static int idRightback = 11;

		public static final List<MotorRole> invertedMotors = List.of(MotorRole.LeftMaster);

		public static final class Odometry {
			public static final double wheelPerimeterMeters = 0.47;
			public static final double trackWidthMeters = 0.5;
			// The transmission denotes how many revolution the motor makes compared to the
			// wheel
			public static final double transmission = 10.71;
			public static final int encoderResolution = 2048;

			public static final double encoderToMetersConversion = wheelPerimeterMeters / transmission
					* encoderResolution;
		}

		public static final class PathWeaver {
			public static final double ksMeters = 0.12091;
			public static final double kvMetersPerSecoond = 2.3501;
			public static final double ka = 0.21997;

			public static final double kMaxVMetersPerSecond = 3.3;
			public static final double kMaxAccMetersPerSecond = 1.2;
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
						List.of(idLeftfront, idRightfront, idLeftback, idRightback),
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

		public static final class Swerve2024 {

			public static final double gearRatio = 5.192308;

			public static final boolean enabled = true;
			public static final double absoluteEncoderZeroPositionTolerance = 0.01;
			public static final boolean joystickYinverted = true;
			public static final boolean joystickXinverted = true;
			public static final double deadBand = 0.18;
			public static final double yOffsetMapperMaxVoltage = 12.5;
			public static final double yOffsetMapperMinVoltage = 9;
			public static final double maxSpeedOfDrive = 25;
			public static final double maxRotationSpeed = 25 * 2 * Math.PI;
			public static final Map<MountingLocations, frc.robot.subsystems.drive.swerve_2024.SwerveModule.Config> swerveModuleConfigs = new HashMap<>();

			public static frc.robot.subsystems.drive.swerve_2024.SwerveModule.Config commonConfigurations = new frc.robot.subsystems.drive.swerve_2024.SwerveModule.Config();

			public static final double absoluteEncoderToMeters = 1;
			public static final double metersToRelativeEncoder = 1;

			private static final List<Integer> motorIds = List.of(
					1, 2, 3, 4,
					11, 12, 13, 14);

			public static final RobotData robotData = new RobotData(
					new HardwareData(
							null,
							Meters.of(0.12 * Math.PI),
							1 / metersToRelativeEncoder),
					new DriveData(
							enabled,
							motorIds,
							List.of()),
					new AutoData(
							null,
							null,
							null,
							null,
							null,
							null,
							null),
					new PidData(null, null, null));

			static {
				addCommonModuleConfigurarions();
				addModuleSpecificConfigurarions();
			}

			private static void addCommonModuleConfigurarions() {
				commonConfigurations.driveMotorTicksPerRotation = 2048.0;
				commonConfigurations.rotationMotorTicksPerRotation = 47.691;
				commonConfigurations.drivePID = new PidValues(0.029, 0, 0);
				commonConfigurations.driveFeedForward = new FeedForwardValues(0.179, 0.270);
				commonConfigurations.drivePID.slotIdX = Optional.of(0);
				commonConfigurations.rotationPID = new PidValues(1.05, 0.01, 1);
				commonConfigurations.rotationPID.slotIdX = Optional.of(0);
				commonConfigurations.wheelCircumference = 0.12 * Math.PI;
				commonConfigurations.maxVelocity = maxSpeedOfDrive;
				commonConfigurations.driveEncoderType = FridoFeedBackDevice.kBuildin;
				commonConfigurations.rotationEncoderType = FridoFeedBackDevice.kBuildin;
				commonConfigurations.driveIdleMode = IdleMode.kBrake;
				commonConfigurations.rotationIdleMode = IdleMode.kBrake;
			}

			public static final double xOffset = 0.275;
			public static final double yOffset = 0.275;

			public static final Translation2d[] SWERVE_MODULE_TRANSLATIONS = {
					new Translation2d(-xOffset, yOffset),
					new Translation2d(-xOffset, -yOffset),
					new Translation2d(xOffset, -yOffset),
					new Translation2d(xOffset, yOffset)
			};

			public static final SwerveModulePosition[] SWERVE_MODULE_POSITIONS = {
					new SwerveModulePosition(SWERVE_MODULE_TRANSLATIONS[0].getNorm(),
							SWERVE_MODULE_TRANSLATIONS[0].getAngle()),
					new SwerveModulePosition(SWERVE_MODULE_TRANSLATIONS[1].getNorm(),
							SWERVE_MODULE_TRANSLATIONS[1].getAngle()),
					new SwerveModulePosition(SWERVE_MODULE_TRANSLATIONS[2].getNorm(),
							SWERVE_MODULE_TRANSLATIONS[2].getAngle()),
					new SwerveModulePosition(SWERVE_MODULE_TRANSLATIONS[3].getNorm(),
							SWERVE_MODULE_TRANSLATIONS[3].getAngle())
			};

			private static FridolinsMotor driveMotorInitializer(int id) {
				var motor = new FridoFalcon500v6(id);
				motor.factoryDefault();
				motor.asTalonFX().getConfigurator().apply(new Slot0Configs().withKP(0.03).withKS(0.18).withKV(0.27));
				return motor;
			}

			private static FridolinsMotor angleMotorInitializer(int id, MotorType motorType) {
				var motor = new FridoCanSparkMax(id, MotorType.kBrushless);
				motor.factoryDefault();
				motor.setSmartCurrentLimit(20, 20);
				motor.getPIDController().setIZone(1.5);
				return motor;
			}

			private static void addModuleSpecificConfigurarions() {
				frc.robot.subsystems.drive.swerve_2024.SwerveModule.Config frontLeftConfig = commonConfigurations
						.clone();
				frontLeftConfig.absoluteEncoderZeroPosition = 0.5348;
				frontLeftConfig.mountingPoint = new Translation2d(-xOffset, yOffset);
				frontLeftConfig.driveMotorInitializer = () -> driveMotorInitializer(1);
				frontLeftConfig.rotationMotorInitializer = () -> angleMotorInitializer(11, MotorType.kBrushless);
				frontLeftConfig.driveMotorInverted = false;
				frontLeftConfig.absoluteEncoderChannel = 0;
				swerveModuleConfigs.put(MountingLocations.FrontLeft, frontLeftConfig);

				frc.robot.subsystems.drive.swerve_2024.SwerveModule.Config frontRightConfig = commonConfigurations
						.clone();
				frontRightConfig.absoluteEncoderZeroPosition = 0.201;
				frontRightConfig.mountingPoint = new Translation2d(-xOffset, -yOffset);
				frontRightConfig.driveMotorInitializer = () -> driveMotorInitializer(2);
				frontRightConfig.rotationMotorInitializer = () -> angleMotorInitializer(12, MotorType.kBrushless);
				frontRightConfig.driveMotorInverted = false;
				frontRightConfig.absoluteEncoderChannel = 1;
				swerveModuleConfigs.put(MountingLocations.FrontRight, frontRightConfig);

				frc.robot.subsystems.drive.swerve_2024.SwerveModule.Config backLeftConfig = commonConfigurations
						.clone();
				backLeftConfig.absoluteEncoderZeroPosition = 0.4605;
				backLeftConfig.mountingPoint = new Translation2d(xOffset, yOffset);
				backLeftConfig.driveMotorInitializer = () -> driveMotorInitializer(3);
				backLeftConfig.rotationMotorInitializer = () -> angleMotorInitializer(13, MotorType.kBrushless);
				backLeftConfig.driveMotorInverted = false;
				backLeftConfig.absoluteEncoderChannel = 2;
				swerveModuleConfigs.put(MountingLocations.BackLeft, backLeftConfig);

				frc.robot.subsystems.drive.swerve_2024.SwerveModule.Config backRightConfig = commonConfigurations
						.clone();
				backRightConfig.absoluteEncoderZeroPosition = 0.115;
				backRightConfig.mountingPoint = new Translation2d(xOffset, -yOffset);
				backRightConfig.driveMotorInitializer = () -> driveMotorInitializer(4);
				backRightConfig.rotationMotorInitializer = () -> angleMotorInitializer(14, MotorType.kBrushless);
				backRightConfig.driveMotorInverted = false;
				backRightConfig.absoluteEncoderChannel = 3;
				swerveModuleConfigs.put(MountingLocations.BackRight, backRightConfig);
			}
		}

		public static final class Swerve2019 {
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
			public static final double maxRotationSpeed = 15 * Math.PI / 16; // at full rotation speed the robot will
																				// turn
																				// by 180 degrees, in rad per second
			public static final Map<MountingLocations, SwerveModule.Config> swerveModuleConfigs = new HashMap<>();

			public static SwerveModule.Config commonConfigurations = new SwerveModule.Config();
			public static double defaultSpeedFactor = 0.3;
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
				commonConfigurations.rotationPID = new PidValues(0.04, 0.0, 0.5);
				commonConfigurations.rotationPID.slotIdX = Optional.of(0);
				commonConfigurations.wheelCircumference = 0.1 * Math.PI;
				commonConfigurations.maxVelocity = maxSpeedOfDrive;
				commonConfigurations.driveEncoderType = FridoFeedBackDevice.kRelative;
				commonConfigurations.rotationEncoderType = FridoFeedBackDevice.kRelative;
				commonConfigurations.limitSwitchPolarity = LimitSwitchPolarity.kNormallyOpen;
			}

			private static FridoTalonSRX angleMotorInitializer(int id, MotorType motorType) {
				FridoTalonSRX motor = new FridoTalonSRX(id);
				motor.factoryDefault();
				return motor;
			}

			private static FridolinsMotor driveMotorInitializer(int id, MotorType motorType) {
				FridoTalonSRX motor = angleMotorInitializer(id, motorType);
				motor.enableForwardLimitSwitch(LimitSwitchPolarity.kDisabled, false);
				motor.enableReverseLimitSwitch(LimitSwitchPolarity.kDisabled, false);
				return motor;
			}

			private static void addModuleSpecificConfigurarions() {
				// TODO: Add limit switch stuff again

				SwerveModule.Config frontLeftConfig = commonConfigurations.clone();
				frontLeftConfig.mountingPoint = new Translation2d(0.32, 0.305);
				frontLeftConfig.driveMotorInitializer = () -> driveMotorInitializer(32, MotorType.kBrushless);
				frontLeftConfig.rotationMotorInitializer = () -> angleMotorInitializer(33, MotorType.kBrushless);
				frontLeftConfig.halSensorPosition = -0;
				swerveModuleConfigs.put(MountingLocations.FrontLeft, frontLeftConfig);

				SwerveModule.Config frontRightConfig = commonConfigurations.clone();
				frontRightConfig.mountingPoint = new Translation2d(-0.32, 0.305);
				frontRightConfig.driveMotorInitializer = () -> driveMotorInitializer(38, MotorType.kBrushless);
				frontRightConfig.rotationMotorInitializer = () -> angleMotorInitializer(39, MotorType.kBrushless);
				frontRightConfig.halSensorPosition = 0;
				swerveModuleConfigs.put(MountingLocations.FrontRight, frontRightConfig);

				SwerveModule.Config backLeftConfig = commonConfigurations.clone();
				backLeftConfig.mountingPoint = new Translation2d(0.32, -0.305);
				backLeftConfig.driveMotorInitializer = () -> driveMotorInitializer(34, MotorType.kBrushless);
				backLeftConfig.rotationMotorInitializer = () -> angleMotorInitializer(35, MotorType.kBrushless);
				backLeftConfig.halSensorPosition = 0;
				swerveModuleConfigs.put(MountingLocations.BackLeft, backLeftConfig);

				SwerveModule.Config backRightConfig = commonConfigurations.clone();
				backRightConfig.mountingPoint = new Translation2d(-0.32, -0.305);
				backRightConfig.driveMotorInitializer = () -> driveMotorInitializer(36, MotorType.kBrushless);
				backRightConfig.rotationMotorInitializer = () -> angleMotorInitializer(37, MotorType.kBrushless);
				backRightConfig.halSensorPosition = 0;
				swerveModuleConfigs.put(MountingLocations.BackRight, backRightConfig);
			}
		}
	}

}
