package frc.robot.abstraction.baseClasses;

import java.util.Optional;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * BTankDrive: Base class for the tank drive. Main functionality defined in BDrive or rather IDrive
 */
public abstract class BTankDrive extends BDrive {

	// Swerve only --> BTankDrive is no swerve
    public Optional<SwerveDriveKinematics> getSwerveKinematics() {
		return Optional.empty();
	}

    public boolean isSwerve() {
		return false;
	}
}
