package frc.robot.abstraction.baseClasses;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Map.Entry;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.fridowpi.utils.Algorithms;
import frc.robot.Constants;
import frc.robot.subsystems.drive.swerve_2024.SwerveKinematics;
import frc.robot.subsystems.visionAutonomous.CustomSwerveDrivePoseEstimator;
import frc.robot.subsystems.visionAutonomous.Visionprocessing;

public abstract class BSwerveDrive extends BDrive {
	// TODO: clean up here
	protected SwerveKinematics<MountingLocations> kinematics;
	protected CustomSwerveDrivePoseEstimator poseEstimator;


	@Override
	public void init() {
		super.init();
		setUpSwerveKinematics(); // Must be called before setUpPoseEstimator()
		setUpPoseEstimator();
	}

	private void setUpSwerveKinematics() {
		Map<MountingLocations, Translation2d> mountingPoints = Constants.SwerveDrive.Swerve2024.swerveModuleConfigs
				.entrySet().stream().map(Algorithms.mapEntryFunction(config -> config.mountingPoint))
				.collect(Collectors.toMap(Entry::getKey, Entry::getValue));
		kinematics = new SwerveKinematics<MountingLocations>(mountingPoints);
	}

	private void setUpPoseEstimator() {
		double[] pos = Visionprocessing.getInstance().getFieldPos();
		poseEstimator = CustomSwerveDrivePoseEstimator.fromFieldPos(pos);
	}

	public DriveOrientation getDriveMode() {
		return DriveOrientation.Backwards;
	}

	public void setDriveMode(DriveOrientation driveMode) {
	}

	public void drive(ChassisSpeeds requesteSpeeds) {
	}

	public void rotateAllModules(double speed) {
	}

	public void resetOdometry() {
		poseEstimator.resetOdometry();
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
	public Optional<SwerveDriveKinematics> getSwerveKinematics() {
		return Optional.of(kinematics);
	}

	@Override
	public Optional<DifferentialDriveKinematics> getDifferentialKinematics() {
		return Optional.empty();
	}

	@Override
	public Optional<DifferentialDriveWheelSpeeds> getDifferentialWheelSpeeds() {
		return Optional.empty();
	}

	// TODO: make optional
	@Override
	public double getLeftEncoderPos() {
		throw new UnsupportedOperationException("Unimplemented method 'getLeftEncoderPos'");
	}

	@Override
	public double getRightEncoderPos() {
		throw new UnsupportedOperationException("Unimplemented method 'getRightEncoderPos'");
	}

	@Override
	public final boolean isSwerve() {
		return true;
	}
}
