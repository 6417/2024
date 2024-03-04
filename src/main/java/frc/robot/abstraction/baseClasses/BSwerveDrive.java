package frc.robot.abstraction.baseClasses;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public abstract class BSwerveDrive extends BDrive {
	// TODO: clean up here

	public DriveOrientation getDriveMode() {
		return DriveOrientation.Backwards;
	}

	public void setDriveMode(DriveOrientation driveMode) {
	}

	public void drive(ChassisSpeeds requesteSpeeds) {
	}

	@Override
	public void drive(double v_x, double v_y, double rot) {
		// TODO convert call to drive(ChassisSpeeds)
		throw new UnsupportedOperationException("Unimplemented method 'drive'");
	}

	public void rotateAllModules(double speed) {
	}

	@Override
	public void stopAllMotors() {
	}

	public void setRotationToHome(MountingLocations moduleLocation) {
	}

	public void setRotationEncoderTicks(MountingLocations mountingLocation, double ticks) {
	}

	public void forEachModule(Consumer<BSwerveModule> consumer) {
	}

	public boolean isModuleZeroed(MountingLocations mountingLocation) {
		return false;
	}

	public final boolean areAllModulesZeroed() {
		for (MountingLocations location : MountingLocations.values()) {
			if (!isModuleZeroed(location)) {
				return false;
			}
		}
		return true;
	}

	public Map<MountingLocations, Boolean> getZeroedModules() {
		Map<MountingLocations, Boolean> result = new HashMap<>();
		forEachModuleEntry(
				labeledModule -> result.put(
						labeledModule.getKey(), true));
		return result;
	}

	public void forEachModuleEntry(
			Consumer<Map.Entry<MountingLocations, BSwerveModule>> consumer) {
	}

	public void withModule(MountingLocations mountingLocation, Consumer<BSwerveModule> consumer) {
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
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'getSwerveKinematics'");
	}

	@Override
	public final boolean isSwerve() {
		return true;
	}
}
