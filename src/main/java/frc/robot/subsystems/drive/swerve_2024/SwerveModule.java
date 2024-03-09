package frc.robot.subsystems.drive.swerve_2024;

import java.util.Collection;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.fridowpi.module.IModule;
import frc.fridowpi.motors.FridoFalcon500v6;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.utils.PidValues;
import frc.fridowpi.utils.Vector2;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BSwerveModule;

public class SwerveModule extends BSwerveModule {
	// Set variables (TODO: Intergrate into RobotData)
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

	private class Motors {
		public FridolinsMotor drive;
		public FridolinsMotor rotation;
		public AnalogEncoder absoluteEncoder;

		public Motors(Config config) {
			drive = config.driveMotorInitializer.get();
			drive.configEncoder(config.driveEncoderType, (int) config.driveMotorTicksPerRotation);
			config.driveSensorInverted.ifPresent(this.drive::setEncoderDirection);
			drive.setInverted(config.driveMotorInverted);
			drive.setIdleMode(IdleMode.kBrake);
			//drive.setPID(config.drivePID);

			rotation = config.rotationMotorInitializer.get();
			rotation.configEncoder(config.rotationEncoderType, (int) config.rotationMotorTicksPerRotation);
			rotation.setIdleMode(IdleMode.kCoast);
			rotation.setPID(config.rotationPID);

			absoluteEncoder = new AnalogEncoder(config.absoluteEncoderChannel);
			absoluteEncoder.setPositionOffset(config.absoluteEncoderZeroPosition);
			zeroRelativeEncoder();
		}

		public void zeroRelativeEncoder() {
			var pos = absoluteEncoder.get();
			var newPos = pos * config.rotationMotorTicksPerRotation;
			rotation.setEncoderPosition(newPos);
		}
	}

	private Motors motors;
	public Config config;
	private SwerveModuleState desiredState = new SwerveModuleState();
	public boolean currentRotationInverted = false;

	public SwerveModule(Config config) {
		this.config = config;
		motors = new Motors(this.config);
	}

	public Vector2 getModuleRotation() {
		return Vector2.fromRad(getModuleRotationAngle());
	}

	public void zeroRelativeEncoder() {
		motors.zeroRelativeEncoder();
	}

	public double getModuleRotationAngle() {
		return Vector2
				.fromRad(((motors.rotation.getEncoderTicks() / config.rotationMotorTicksPerRotation) * Math.PI * 2)
						% (Math.PI * 2))
				.toRadians();
	}

	public double getRawModuleRotationAngle() {
		return (motors.rotation.getEncoderTicks() / config.rotationMotorTicksPerRotation) * Math.PI * 2;
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
				+ steeringDirection * (angleDelta / (Math.PI * 2)) * config.rotationMotorTicksPerRotation;
	}

	private double meterPerSecondToDriveMotorEncoderVelocityUnits(double speedMs) {
		return (speedMs / config.wheelCircumference) * config.driveMotorTicksPerRotation;
	}

	private double driveMotorEncoderVelocityToPercent(double encoderSpeed) {
		return encoderSpeed / meterPerSecondToDriveMotorEncoderVelocityUnits(config.maxVelocity);
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
		((FridoFalcon500v6) motors.drive).asTalonFX().setControl(new VelocityVoltage(speedFactor * desiredState.speedMetersPerSecond));
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
		return motors.absoluteEncoder.get();
	}

	public void stopAllMotors() {
		motors.drive.set(0.0);
		motors.rotation.set(0.0);
	}

	public void setCurrentRotationToEncoderHome() {
		motors.rotation.setEncoderPosition(0);
		motors.absoluteEncoder.setPositionOffset(motors.absoluteEncoder.getAbsolutePosition());
	}

	public void invertRotationDirection() {
		currentRotationInverted = !currentRotationInverted;
	}

	// Getters and setters
	public SwerveModuleState getDesiredModuleState() {
		return desiredState;
	}

	@Override
	public Config getConfig() {
		return config;
	}

	IdleMode mode = IdleMode.kCoast;
	double absPos = 0;

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Desired speed", () -> desiredState.speedMetersPerSecond, null);
		builder.addDoubleProperty("Desired angle", desiredState.angle::getDegrees, null);
		builder.addDoubleProperty("Current speed", this::getSpeed, null);
		builder.addDoubleProperty("Current angel", () -> getModuleRotationAngle() * 180 / Math.PI, null);
		builder.addDoubleProperty("Rotation Encoder Ticks", motors.rotation::getEncoderTicks, null);
		builder.addDoubleProperty("Absolute Encoder", motors.absoluteEncoder::get, null);
		builder.addDoubleProperty("Absolute Pos", motors.absoluteEncoder::getAbsolutePosition, null);
		builder.addDoubleProperty("Velocity Error", () -> Math.abs(getSpeed()) - Math.abs(desiredState.speedMetersPerSecond * 0.075), null);
		builder.addDoubleProperty("Set abs enc", () -> absPos, val -> {
			absPos = val;
			motors.absoluteEncoder.reset();
		});
			// motors.absoluteEncoder.setPositionOffset(val);});
		builder.addBooleanProperty("Zero Module", () -> false, _ignore -> zeroRelativeEncoder());

		builder.addDoubleProperty("Target", motors.rotation::getPidTarget, null);
		builder.addBooleanProperty("Coast", () -> mode == IdleMode.kCoast, val -> mode = val? IdleMode.kCoast: IdleMode.kBrake);
	}

	@Override
	public void zeroAbsoluteEncoder() {
		motors.absoluteEncoder.reset();
	}

	@Override
	public void setIdleMode(IdleMode mode) {
		/* This function shell not be used at all */
		motors.drive.setIdleMode(mode);
		motors.rotation.setIdleMode(mode);
		this.mode = mode;
	}

	@Override
	public SwerveModulePosition getOdometryPos() {
		return new SwerveModulePosition(
				motors.drive.getEncoderTicks() / config.driveMotorTicksPerRotation * 65 *  config.wheelCircumference * Constants.SwerveDrive.Swerve2024.gearRatio, new Rotation2d(Radians.of(getModuleRotationAngle())));
	}

	@Override
	public Collection<IModule> getAllSubModules() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'getAllSubModules'");
	}

	@Override
	public Collection<IModule> getSubModules() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'getSubModules'");
	}

	@Override
	public void registerSubmodule(IModule... subModule) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'registerSubmodule'");
	}

	@Override
	public void init() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'init'");
	}

	@Override
	public boolean isInitialized() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'isInitialized'");
	}

	@Override
	public double getWheelSpeed() {
		return motors.drive.getEncoderVelocity() * Constants.SwerveDrive.Swerve2024.gearRatio * config.wheelCircumference;
	}
}
