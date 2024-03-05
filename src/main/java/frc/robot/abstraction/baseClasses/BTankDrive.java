package frc.robot.abstraction.baseClasses;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.joystick.JoystickBindings2024;
import frc.robot.subsystems.drive.tankdrive.MotorSet;
import frc.robot.subsystems.drive.tankdrive.MotorSet.MotorRole;
import frc.robot.subsystems.visionAutonomous.TankDrivePoseEstimator;

/**
 * BTankDrive: Base class for the tank drive. Main functionality defined in BDrive or rather IDrive
 */
public abstract class BTankDrive extends BDrive {

	protected MotorSet motors;
	protected DifferentialDriveKinematics kinematics;
	protected TankDrivePoseEstimator poseEstimator;

	@Override
	public void drive(ChassisSpeeds chassisSpeeds) {
		var x = chassisSpeeds.vxMetersPerSecond;
		var y = chassisSpeeds.vyMetersPerSecond;
		drive(x, 0, y);
	}

	@Override
	public <T> FridolinsMotor getMotor(T motor) {
		assert motor instanceof MotorRole;
		return motors.getMotor((MotorRole)motor);
	}

	// Swerve only --> BTankDrive is no swerve
    public Optional<SwerveDriveKinematics> getSwerveKinematics() {
		return Optional.empty();
	}

	public void resetOdometry() {
		poseEstimator.reset();
	}

    public SwerveModulePosition[] getOdometryPoses() {
		throw new UnsupportedOperationException("Unimplemented method 'getOdometryPoses' for tankdrive");
	}

    public boolean isSwerve() {
		return false;
	}

	@Override
	public List<Binding> getMappings() {
		return JoystickBindings2024.getBindingsTankdriveLogitech();
	}
}
