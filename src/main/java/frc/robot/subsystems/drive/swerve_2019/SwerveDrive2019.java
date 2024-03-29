package frc.robot.subsystems.drive.swerve_2019;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.fridowpi.utils.Algorithms;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDrive.Swerve2019;
import frc.robot.abstraction.baseClasses.BDrive.DriveOrientation;
import frc.robot.abstraction.baseClasses.BDrive.MountingLocations;
import frc.robot.commands.drive.commands_2019.DriveCommand2019;

public class SwerveDrive2019 extends SwerveDriveBase {
	private DriveOrientation driveMode = DriveOrientation.Forwards;
	private static SwerveDriveBase instance = null;
	private SwerveKinematics<MountingLocations> kinematics;
	private Map<MountingLocations, SwerveModule> modules = new HashMap<>();
	private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();
	private double speedFactor = Swerve2019.defaultSpeedFactor;

	private void setUpSwerveKinematics() {
		Map<MountingLocations, Translation2d> mountingPoints = Swerve2019.swerveModuleConfigs
				.entrySet().stream().map(Algorithms.mapEntryFunction(config -> config.mountingPoint))
				.collect(Collectors.toMap(Entry::getKey, Entry::getValue));
		kinematics = new SwerveKinematics<MountingLocations>(mountingPoints);
	}

	private void setUpSwerveModules() {
		modules = Swerve2019.swerveModuleConfigs.entrySet().stream()
				.map(Algorithms.mapEntryFunction(SwerveModule::new))
				.collect(Collectors.toMap(Entry::getKey, Entry::getValue));
		forEachModuleEntry(moduleEntry -> Shuffleboard.getTab("Drive")
				.add("SwerveModule" + moduleEntry.getKey().toString(), moduleEntry.getValue()));
	}

	private SwerveDrive2019() {
		setUpSwerveModules();
		setUpSwerveKinematics();
	}

	public static SwerveDriveBase getInstance() {
		if (instance == null)
			if (Constants.SwerveDrive.Swerve2019.enabled) {
				instance = new SwerveDrive2019();
				instance.setDefaultCommand(new DriveCommand2019());
			} else {
				instance = new SwerveDriveBase();
			}
		return instance;
	}

	@Override
	public boolean isModuleZeroed(MountingLocations mountingLocation) {
		return modules.get(mountingLocation).hasEncoderBeenZeroed();
	}

	@Override
	public void withModule(MountingLocations mountingLocation, Consumer<SwerveModule> consumer) {
		consumer.accept(modules.get(mountingLocation));
	}

	private double getMaxSpeed(Map<MountingLocations, SwerveModuleState> states) {
		return states.values().stream().max(Comparator.comparing(state -> state.speedMetersPerSecond))
				.get().speedMetersPerSecond;
	}

	private Map<MountingLocations, SwerveModuleState> normalizeStates(
			Map<MountingLocations, SwerveModuleState> states) {
		if (getMaxSpeed(states) > Constants.SwerveDrive.Swerve2019.maxSpeedOfDrive * speedFactor)
			return states.entrySet().stream()
					.map(Algorithms.mapEntryFunction(
							Algorithms.mapSwerveModuleStateSpeed(speed -> speed / getMaxSpeed(states))))
					.map(Algorithms.mapEntryFunction(
							Algorithms.mapSwerveModuleStateSpeed(speed -> speed * speedFactor)))
					.collect(Collectors.toMap(Entry::getKey, Entry::getValue));
		return states;
	}

	@Override
	public void drive(ChassisSpeeds requestedMovement) {
		currentChassisSpeeds = requestedMovement;
		Map<MountingLocations, SwerveModuleState> states = kinematics
				.toLabledSwerveModuleStates(currentChassisSpeeds);
		states = normalizeStates(states);

		states.entrySet().forEach(
				(Entry<MountingLocations, SwerveModuleState> labeledState) -> modules
						.get(labeledState.getKey()).setDesiredState(labeledState.getValue()));

		forEachModule(module -> module.drive(speedFactor));
	}

	@Override
	public void rotateAllModules(double speed) {
		forEachModule(module -> module.rotateModule(speed));
	}

	@Override
	public Map<MountingLocations, Boolean> areHalSensoredOfMoudlesTriggered() {
		Map<MountingLocations, Boolean> result = new HashMap<>();
		forEachModuleEntry(
				labeledModule -> result.put(
						labeledModule.getKey(),
						labeledModule.getValue().isHalSensorTriggered()));
		return result;
	}

	@Override
	public void setCurrentModuleRotatoinToHome(MountingLocations moduleLocation) {
		modules.get(moduleLocation).setCurrentRotationToEncoderHome();
	}

	public static double joystickInputToMetersPerSecond(double joystickValue) {
		return joystickValue * Constants.SwerveDrive.Swerve2019.maxSpeedOfDrive;
	}

	public static double joystickInputToRadPerSecond(double joystickValue) {
		return joystickValue * Constants.SwerveDrive.Swerve2019.maxRotationSpeed;
	}

	@Override
	public void forEachModule(Consumer<SwerveModule> consumer) {
		modules.values().stream().forEach(consumer);
	}

	@Override
	public void stopAllMotors() {
		forEachModule(module -> module.stopAllMotors());
	}

	@Override
	public boolean areAllModulesZeroed() {
		return modules.values()
				.stream()
				.map(module -> module.hasEncoderBeenZeroed())
				.reduce(true, (previousZeroed, currentZeroed) -> previousZeroed && currentZeroed);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		// builder.addDoubleProperty("odometry x",() -> swerveDrivePoseEstimator.getInstance().swerveDrivePoseEstimator.getEstimatedPosition().getX(), null);
		// builder.addDoubleProperty("odometry y",() -> SwerveDrivePoseEstimator.getInstance().swerveDrivePoseEstimator.getEstimatedPosition().getY(), null);
		// builder.addDoubleProperty("odometry rot",() -> swerveDrivePoseEstimator.getInstance().swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), null);
	}

	public SwerveDriveKinematics getKinematics() {
		return kinematics;
	}

	@Override
	public void forEachModuleEntry(
			Consumer<Map.Entry<MountingLocations, SwerveModule>> consumer) {
		modules.entrySet().stream().forEach(consumer);
	}

	@Override
	public void setModuleRotationEncoderTicks(MountingLocations mountingLocation, double ticks) {
		modules.get(mountingLocation).setRotationEncoderTicks(ticks);
	}

	@Override
	public void setSpeedFactor(double speedFactor) {
		assert speedFactor > 0.0 : "speedFactor must be grater than zero";
		this.speedFactor = speedFactor;
	}

	@Override
	public DriveOrientation getDriveMode() {
		return driveMode;
	}

	@Override
	public void setDriveMode(DriveOrientation driveMode) {
		this.driveMode = driveMode;
	}

	@Override
	public SwerveModulePosition[] getOdometryPoses(){
		ArrayList<SwerveModulePosition> pos = new ArrayList<SwerveModulePosition>();
		for (var module:modules.values()){
			pos.add(module.getOdometryPos());
		}
		return pos.toArray(SwerveModulePosition[]::new);
	}

	@Override
	public void configureButtonBindings(Joystick joystick) {
	}
}
