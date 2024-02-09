package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.fridowpi.motors.FridoTalonSRX;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.LimitSwitchPolarity;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.subsystems.drive.swerve.SwerveModule;

public class Constants {

    public static final boolean driveEnabled = true;

    public static final class Joystick {
        public static final IJoystickId id = () -> 0;
    }

    public static final class Testchassi{
		public static final boolean driveEnabled = false;

        final public static int idRigthfront = 13;
        final public static int idLeftfront = 12;
        final public static int idRigthback = 11;
        final public static int idLeftback = 0;

        public static final class Odometry {
            public static final double wheelPerimeter = 0.47;
            // The transmission denotes how many revolution the motor makes compared to the wheel
            public static final double transmission = 10.71;
            public static final int encoderResolution = 2048;

            public static final double shaftRotationsToMeters = wheelPerimeter / transmission;
            public static final double encoderToMeters = shaftRotationsToMeters / encoderResolution;
            public static final double trackWidthMeters = 0.5;
        }

        public static final class PathWeaver {
            //public static final double ksMeters = 0.10289;
            //public static final double kvMetersPerSecoond = 2.679/2;
            //public static final double ka = 0.40922;

            public static final double kMaxVMetersPerSecond = 1;            //own constraints
            public static final double kMaxAccMetersPerSecond = 0.8; //0.9
            public static final double kMaxCentripetalAcceleration = 0;

            public static final double kRamsetB = 0;
            public static final double kRamseteZeta = 0;

            public static final double kP = 0.62585;// old value 0.36205;
            public static final double kI = 0;
            public static final double kD = 0;
        }


        //final public static double kMaxVMetersPerSecond = 0;
        //final public static double kMaxAccMetersPerSecond = 0;
        //final public static DifferentialDriveKinematics kDriveKinematics;

        final public static double ksVolts = 0.102; //old value 0.4
        final public static double kvVoltSevondsPerMeter = 2.679; //old value 1.98
        final public static double kaVoltSecondsSquaredPerMeter = 0.409; //old value 0.6
        final public static double kPDriveVel = 0.62585; //old value 1.72585 from sysid 0.62585

        final public static double ticsToMeter = 0.042;
    }

    public static final class Tankdrive {

    }

    public static final class Swervedrive {
        public static final class Drive {
            public static final double kMaxSpeedMetersPerSecond = 0;
            public static final double kMaxAccelerationMetersPerSecondSquared = 0;

            public static final double gearRatio = 1.0 / 5.192308;
        }
    }

    public static final class Shooter {
        public static final double OptimalAmpSpeed = 0.3;
        public static final double OptimalSpeakerSpeed = 0.9;
        public static final double OptimalIntakeSpeed = -0.6;
    }

    public static final class Sysid {
        public static final boolean isTuning = false;
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

        @SuppressWarnings("all")
        public static final boolean enabled = driveEnabled;
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
        public static double fullSpeed =  1.0;

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
            //         new Vector2(-1 / Math.sqrt(2), -1 / Math.sqrt(2)),
            //         new Vector2(-1 / Math.sqrt(2), 1 / Math.sqrt(2)) };
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
