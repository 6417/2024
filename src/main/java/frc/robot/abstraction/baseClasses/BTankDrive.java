package frc.robot.abstraction.baseClasses;

import java.util.Optional;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.subsystems.drive.tankdrive.MotorSet;
import frc.robot.subsystems.drive.tankdrive.MotorSet.MotorRole;

/**
 * BTankDrive: Base class for the tank drive. Main functionality defined in BDrive or rather IDrive
 */
public abstract class BTankDrive extends BDrive {

	protected MotorSet motors;

	@Override
	public <T> FridolinsMotor getMotor(T motor) {
		assert motor instanceof MotorRole;
		return motors.getMotor((MotorRole)motor);
	}

	// Swerve only --> BTankDrive is no swerve
    public Optional<SwerveDriveKinematics> getSwerveKinematics() {
		return Optional.empty();
	}

    public boolean isSwerve() {
		return false;
	}
}
