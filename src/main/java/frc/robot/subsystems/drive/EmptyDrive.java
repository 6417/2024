package frc.robot.subsystems.drive;

import static frc.robot.Utils.log;
import static frc.robot.Utils.logerr;

import java.util.Optional;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.abstraction.baseClasses.BDrive;

/**
 * EmptyDrive
 */
public class EmptyDrive extends BDrive {

	@Override
	public void drive(double v_x, double v_y, double rot) {
	}

	@Override
	public void drive(ChassisSpeeds chassisSpeeds) {
	}

	@Override
	public void setVolts(double leftvolts, double rigthvolts) {
	}

	@Override
	public void driveToPos(Pose2d pos) {
		log("<Drive> driveToPos" + pos);
	}

	@Override
	public void setSpeedFactor(double speedFactor) {
		log("<Drive> new speed factor: " + speedFactor);
	}

	@Override
	public void stopAllMotors() {
		log("<Drive> motors stopped");
	}

	@Override
	public void setIdleMode(IdleMode mode) {
		log("<Drive> idle mode set to " + mode);
	}

	@Override
	public Command sysIdQuasistatic(Direction direction) {
		log("<Drive> sysIdQuasistatic started");
		return new InstantCommand();
	}

	@Override
	public Command sysIdDynamic(Direction _direction) {
		log("<Drive> sysIdDynamic started");
		return new InstantCommand();
	}

	@Override
	public <T> FridolinsMotor getMotor(T motor) {
		logerr("<Drive> Tried to access a non-existent motor: " + motor);
		return null;
	}

	@Override
	public Pose2d getPos() {
		return new Pose2d();
	}

	@Override
	public double getLeftEncoderPos() {
		return 0;
	}

	@Override
	public double getRightEncoderPos() {
		return 0;
	}

	@Override
	public Optional<DifferentialDriveKinematics> getDifferentialKinematics() {
		return Optional.empty();
	}

	@Override
	public Optional<DifferentialDriveWheelSpeeds> getDifferentialWheelSpeeds() {
		return Optional.empty();
	}

	@Override
	public Optional<SwerveDriveKinematics> getSwerveKinematics() {
		return Optional.empty();
	}

	@Override
	public boolean isSwerve() {
		return false;
	}

	@Override
	public void zeroAbsoluteEncoders() {
		log("<Drive> zeroAbsoluteEncoders");
	}

	@Override
	public void zeroRelativeEncoders() {
		log("<Drive> zeroRelativeEncoders");
	}

	@Override
	public SwerveModulePosition[] getOdometryPoses() {
		log("<Drive> getOdometryPoses");
		return new SwerveModulePosition[4];
	}

	@Override
	public void resetOdometry() {
	}
}
