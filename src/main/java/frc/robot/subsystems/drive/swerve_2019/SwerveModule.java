package frc.robot.subsystems.drive.swerve_2019;

import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.fridowpi.motors.FridoFalcon500;
import frc.fridowpi.motors.FridoFalcon500v6;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.FridolinsMotor.LimitSwitchPolarity;
import frc.fridowpi.motors.utils.PidValues;
import frc.fridowpi.utils.Vector2;

public class SwerveModule implements Sendable {
    private boolean isEncoderZeroed = false;

    public static class Config implements Cloneable {
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
        public double halSensorPosition;
        public boolean limitModuleStates;
        public LimitSwitchPolarity limitSwitchPolarity;

        @Override
        public Config clone() {
            try {
                return (Config) super.clone();
            } catch (CloneNotSupportedException e) {
                Config copy = new Config();
                copy.driveMotorInitializer = driveMotorInitializer;
                copy.rotationMotorInitializer = rotationMotorInitializer;
                copy.drivePID = drivePID.clone();
                copy.rotationPID = rotationPID.clone();
                copy.rotationMotorTicksPerRotation = rotationMotorTicksPerRotation;
                copy.driveMotorTicksPerRotation = driveMotorTicksPerRotation;
                copy.wheelCircumference = wheelCircumference;
                copy.mountingPoint = new Translation2d(mountingPoint.getX(), mountingPoint.getY());
                copy.driveMotorInverted = driveMotorInverted;
                copy.halSensorPosition = halSensorPosition;
                copy.limitModuleStates = limitModuleStates;
                copy.limitSwitchPolarity = limitSwitchPolarity;
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

        public Motors(FridolinsMotor drive, FridoFeedBackDevice driveEncoderType, boolean driveMotorInverted,
                Optional<Boolean> driveSensorInverted, FridolinsMotor rotation,
                FridoFeedBackDevice rotationEncoderType, LimitSwitchPolarity limitSwitchPolarity) {
            this.drive = drive;
            this.rotation = rotation;
            this.drive.configEncoder(driveEncoderType, (int) driveMotorTicksPerRotation);
            this.rotation.configEncoder(rotationEncoderType, (int) rotationMotorTicksPerRotation);
            driveSensorInverted.ifPresent(this.drive::setEncoderDirection);
            this.drive.setInverted(driveMotorInverted);
            this.rotation.enableForwardLimitSwitch(limitSwitchPolarity, true);
            this.drive.setIdleMode(IdleMode.kBrake);
            this.rotation.setIdleMode(IdleMode.kBrake);

            var config = ((FridoFalcon500v6) this.drive).asTalonFX().getConfigurator();
            var limit = new CurrentLimitsConfigs()
                                    .withStatorCurrentLimit(55)
                                    .withStatorCurrentLimitEnable(true)
                                    .withSupplyCurrentLimit(50)
                                    .withSupplyCurrentThreshold(50)
                                    .withSupplyCurrentLimitEnable(true);

            config.apply(limit);
            }
    }

    private Motors motors;
    private SwerveModuleState desiredState = new SwerveModuleState();
    public final double halSensorPosition;
    public final boolean limitedModuleStates;
    public boolean currentRotationInverted = false;

    public SwerveModule(Config config) {
        motors = new Motors(config.driveMotorInitializer.get(), config.driveEncoderType, config.driveMotorInverted,
                config.driveSensorInverted, config.rotationMotorInitializer.get(), config.rotationEncoderType,
                config.limitSwitchPolarity);
        motors.drive.setPID(config.drivePID);
        motors.rotation.setPID(config.rotationPID);
        motors.driveMotorTicksPerRotation = config.driveMotorTicksPerRotation;
        motors.rotationMotorTicksPerRotation = config.rotationMotorTicksPerRotation;
        motors.wheelCircumference = config.wheelCircumference;
        motors.maxVelocity = config.maxVelocity;
        halSensorPosition = config.halSensorPosition;
        limitedModuleStates = config.limitModuleStates;
    }

	public SwerveModulePosition getOdometryPos() {
		return new SwerveModulePosition(motors.drive.getEncoderTicks() * motors.driveMotorTicksPerRotation, new Rotation2d(Radians.of(getModuleRotationAngle())));
	}

    public Vector2 getModuleRotation() {
        return Vector2.fromRadians(getModuleRotationAngle());
    }

    public double getModuleRotationAngle() {
        return Vector2
                .fromRadians(((motors.rotation.getEncoderTicks() / motors.rotationMotorTicksPerRotation) * Math.PI * 2)
                        % (Math.PI * 2))
                .getAngleAsRadians();
    }

    public double getRawModuleRotationAngle() {
        return (motors.rotation.getEncoderTicks() / motors.rotationMotorTicksPerRotation) * Math.PI * 2;
    }

    public Vector2 getTargetVector() {
        return Vector2.fromRadians(desiredState.angle.getRadians());
    }

    private double angleToRotationMotorEncoderTicks(double angle) {
        double angleDelta = Math.acos(getModuleRotation().dot(Vector2.fromRadians(angle)));
        if (currentRotationInverted)
            angleDelta = Math.PI * 2 + angleDelta;
        double steeringDirection = Math.signum(getModuleRotation().cross(Vector2.fromRadians(angle)));
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

    public double getWheelDistMeter(){
        return motors.drive.getEncoderTicks() / motors.driveMotorTicksPerRotation * motors.wheelCircumference;
    }

    public void setDesiredState(SwerveModuleState state) {
		var dst = Vector2.fromRadians(state.angle.getRadians());
		var src = getModuleRotation();
		if (src.dot(dst) < 0) {
			state.angle = state.angle.rotateBy(Rotation2d.fromDegrees(180));
			state.speedMetersPerSecond *= -1;
		}
		this.desiredState = state;
    }

    public void enableLimitSwitch() {
        motors.rotation.enableForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen, true);
    }

    public void disableLimitSwitch() {
        motors.rotation.enableForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen, false);
    }

    public void drive(double speedFactor) {
        motors.rotation.setPosition(angleToRotationMotorEncoderTicks(desiredState.angle.getRadians()));
        motors.drive.setVelocity(desiredState.speedMetersPerSecond * speedFactor);
    }

    public boolean isHalSensorTriggered() {
        return motors.rotation.isForwardLimitSwitchActive();
    }

    public void setDriveMotorSpeed(double velocity) {
        motors.drive.setVelocity(velocity);
    }

    public void rotateModule(double speed) {
        motors.rotation.set(speed);
    }

    public void stopDriveMotor() {
        motors.drive.set(0.0);
    }

    public void stopRotationMotor() {
        motors.rotation.set(0.0);
    }

    public double getRotationEncoderTicks() {
        return motors.rotation.getEncoderTicks();
    }

    public void setDesiredRotationMotorTicks(double position) {
        motors.rotation.setPosition(position);
    }

    public void stopAllMotors() {
        stopDriveMotor();
        stopRotationMotor();
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

    public void setEncoderZeroedFalse() {
        isEncoderZeroed = false;
    }

    public SwerveModuleState getDesiredModuleState() {
        return desiredState;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Desired state speed", () -> desiredState.speedMetersPerSecond, null);
        builder.addDoubleProperty("Desired state angle", () -> desiredState.angle.getDegrees(), null);
        builder.addDoubleProperty("Module angel", () -> getModuleRotationAngle() * 360 / (Math.PI * 2), null);
        builder.addDoubleProperty("Moudle speed", () -> getSpeed(), null);
        builder.addDoubleProperty("Module Rotation Encoder Ticks", motors.rotation::getEncoderTicks, null);
        builder.addBooleanProperty("forward limit switch", motors.rotation::isForwardLimitSwitchActive, null);
        builder.addBooleanProperty("Module Zeroed", this::hasEncoderBeenZeroed, null);

        builder.addDoubleProperty("Target", motors.rotation::getPidTarget, null);
    }

    public void setRotationEncoderTicks(double ticks) {
        isEncoderZeroed = true;
        motors.rotation.setEncoderPosition(ticks);
    }
}
