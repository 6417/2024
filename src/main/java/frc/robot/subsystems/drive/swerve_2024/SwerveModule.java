package frc.robot.subsystems.drive.swerve_2024;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.utils.PidValues;
import frc.fridowpi.utils.Vector2;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BSwerveModule;

public class SwerveModule extends BSwerveModule {
	private boolean isEncoderZeroed = false;

	public static class Config implements Cloneable {
		public int absoluteEncoderChannel;
		public double absoluteEncoderZeroPosition;
		public Supplier<FridolinsMotor> driveMotorInitializer;
		public Supplier<FridolinsMotor> rotationMotorInitializer;
		public PidValues drivePID;
		public PidValues rotationPID;
		public double rotationMotorTicksPerRotation;
		public double driveMotorTicksPerRotation;
		public double wheelCircumference; // in meter
		public Translation2d mountingPoint; // in meter
		public double maxVelocity; // in drive motor encoder velocity units
		public FridoFeedBackDevice driveEncoderType;
		public FridoFeedBackDevice rotationEncoderType;
		public Optional<Boolean> driveSensorInverted = Optional.empty();
		public boolean driveMotorInverted;

		@Override
		public Config clone() {
			try {
				return (Config) super.clone();
			} catch (CloneNotSupportedException e) {
				Config copy = new Config();
				copy.absoluteEncoderChannel = absoluteEncoderChannel;
				copy.absoluteEncoderZeroPosition = absoluteEncoderZeroPosition;
				copy.driveMotorInitializer = driveMotorInitializer;
				copy.rotationMotorInitializer = rotationMotorInitializer;
				copy.drivePID = drivePID.clone();
				copy.rotationPID = rotationPID.clone();
				copy.rotationMotorTicksPerRotation = rotationMotorTicksPerRotation;
				copy.driveMotorTicksPerRotation = driveMotorTicksPerRotation;
				copy.wheelCircumference = wheelCircumference;
				copy.mountingPoint = new Translation2d(mountingPoint.getX(), mountingPoint.getY());
				copy.driveMotorInverted = driveMotorInverted;
				return copy;
			}
		}
	}

	private static class Motors {
		public FridolinsMotor drive;
		public FridolinsMotor rotation;
		public double rotationMotorTicksPerRotation;
		public double driveMotorTicksPerRotation;
		public double wheelCircumference;
		public double maxVelocity;

		public Motors(Config config) {
			// Create motors
			drive = config.driveMotorInitializer.get();
			rotation = config.rotationMotorInitializer.get();

			// Set variables (TODO: Intergrate into RobotData)
			driveMotorTicksPerRotation = config.driveMotorTicksPerRotation;
			rotationMotorTicksPerRotation = config.rotationMotorTicksPerRotation;
			wheelCircumference = config.wheelCircumference;
			maxVelocity = config.maxVelocity;

			// Configure motors
			drive.configEncoder(config.driveEncoderType, (int) driveMotorTicksPerRotation);
			rotation.configEncoder(config.rotationEncoderType, (int) rotationMotorTicksPerRotation);
			config.driveSensorInverted.ifPresent(this.drive::setEncoderDirection);
			drive.setInverted(config.driveMotorInverted);
			drive.setIdleMode(IdleMode.kCoast);
			rotation.setIdleMode(IdleMode.kBrake);
			drive.setPID(config.drivePID);
			rotation.setPID(config.rotationPID);
		}
	}

	private Motors motors;
	private Config config;
	private SwerveModuleState desiredState = new SwerveModuleState();
	public boolean currentRotationInverted = false;
	public AnalogEncoder absoluteEncoder;

	public SwerveModule(Config config) {
		motors = new Motors(config);
		this.config = config;
		absoluteEncoder = new AnalogEncoder(config.absoluteEncoderChannel);
	}

	public Vector2 getModuleRotation() {
		return Vector2.fromRad(getModuleRotationAngle());
	}

	public double getModuleRotationAngle() {
		return Vector2
				.fromRad(((motors.rotation.getEncoderTicks() / motors.rotationMotorTicksPerRotation) * Math.PI * 2)
						% (Math.PI * 2))
				.toRadians();
	}

	public double getRawModuleRotationAngle() {
		return (motors.rotation.getEncoderTicks() / motors.rotationMotorTicksPerRotation) * Math.PI * 2;
	}

	public Vector2 getTargetVector() {
		return Vector2.fromRad(desiredState.angle.getRadians());
	}

	private double angleToRotationMotorEncoderTicks(double angle) {
		double angleDelta = Math.acos(getModuleRotation().dot(Vector2.fromRad(angle)));
		if (currentRotationInverted)
			angleDelta = Math.PI * 2 + angleDelta;
		double steeringDirection = Math.signum(getModuleRotation().cross(Vector2.fromRad(angle)));
		return motors.rotation.getEncoderTicks()
				+ steeringDirection * (angleDelta / (Math.PI * 2)) * motors.rotationMotorTicksPerRotation;
	}

	private double meterPerSecondToDriveMotorEncoderVelocityUnits(double speedMs) {
		return (speedMs / motors.wheelCircumference) * motors.driveMotorTicksPerRotation;
	}

	private double driveMotorEncoderVelocityToPercent(double encoderSpeed) {
		return encoderSpeed / meterPerSecondToDriveMotorEncoderVelocityUnits(motors.maxVelocity);
	}

	public double getSpeed() {
		return motors.drive.getEncoderVelocity();
	}

	public void setDesiredState(SwerveModuleState state) {
		var dst = Vector2.fromRad(state.angle.getRadians());
		var src = getModuleRotation();
		if (src.dot(dst) < 0) {
			state.angle = state.angle.rotateBy(Rotation2d.fromDegrees(180));
			state.speedMetersPerSecond *= -1;
		}
		this.desiredState = state;
	}

	@Override
	public void driveForward(double speedFactor) {
		motors.rotation.setPosition(angleToRotationMotorEncoderTicks(desiredState.angle.getRadians()));
		motors.drive.setVelocity(desiredState.speedMetersPerSecond * speedFactor);
	}

	public void setDriveMotorSpeed(double velocity) {
		motors.drive.setVelocity(velocity);
	}

	@Override
	public void rotate(double speed) {
		motors.rotation.set(speed);
	}

	public void setDesiredRotationMotorTicks(double position) {
		motors.rotation.setPosition(position);
	}

	public double getRotationEncoderTicks() {
		return motors.rotation.getEncoderTicks();
	}

	public double getAbsoluteEncoderTicks() {
		return absoluteEncoder.get();
	}

	public void stopAllMotors() {
		motors.drive.set(0.0);
		motors.rotation.set(0.0);
	}

	public boolean hasEncoderBeenZeroed() {
		return isEncoderZeroed;
	}

	public void setCurrentRotationToEncoderHome() {
		isEncoderZeroed = true;
		motors.rotation.setEncoderPosition(0);
	}

	public void invertRotationDirection() {
		currentRotationInverted = !currentRotationInverted;
	}

	// Getters and setters
	public void setEncoderZeroedFalse() {
		isEncoderZeroed = false;
	}

	public SwerveModuleState getDesiredModuleState() {
		return desiredState;
	}

	@Override
	public void setRotationEncoderTicks(double ticks) {
		isEncoderZeroed = true;
		motors.rotation.setEncoderPosition(ticks);
	}

	@Override
	public boolean isAtZero() {
		var min = config.absoluteEncoderZeroPosition - Constants.SwerveDrive.Swerve2024.absoluteEncoderZeroPositionTolerance;
		var max = config.absoluteEncoderZeroPosition + Constants.SwerveDrive.Swerve2024.absoluteEncoderZeroPositionTolerance;
		return absoluteEncoder.getAbsolutePosition() > min && absoluteEncoder.getAbsolutePosition() < max;
	}

	@Override
	public Config getConfig() {
		return config;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Desired speed", () -> desiredState.speedMetersPerSecond, null);
		builder.addDoubleProperty("Desired angle", desiredState.angle::getDegrees, null);
		builder.addDoubleProperty("Current speed", this::getSpeed, null);
		builder.addDoubleProperty("Current angel", () -> getModuleRotationAngle() * 180 / Math.PI, null);
		builder.addDoubleProperty("Rotation Encoder Ticks", motors.rotation::getEncoderTicks, null);
		builder.addDoubleProperty("Absolute Encoder Ticks", absoluteEncoder::getAbsolutePosition, null);
		builder.addBooleanProperty("Is around zero", this::isAtZero, null);
		builder.addBooleanProperty("Module Zeroed", this::hasEncoderBeenZeroed, null);

		builder.addDoubleProperty("Target", motors.rotation::getPidTarget, null);
	}

	@Override
	public void setIdleMode(IdleMode mode) {
		motors.drive.setIdleMode(mode);
	}
}
