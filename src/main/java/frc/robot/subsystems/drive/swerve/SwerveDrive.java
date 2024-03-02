package frc.robot.subsystems.drive.swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.sensors.FridoNavx;
import frc.fridowpi.utils.Algorithms;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConsts.MountingLocations;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.SetSpeedFactor;
import frc.robot.commands.drive.ZeroEncoders;
import frc.robot.subsystems.vision_autonomous.SwervdriveAuto;
import frc.robot.subsystems.vision_autonomous.Swervdrive_poseestimator;

public class SwerveDrive extends SwerveDriveBase {
	private DriveOrientation driveMode = DriveOrientation.ShooterBack;
	private static SwerveDriveBase instance = null;
	private SwerveKinematics<Constants.SwerveDriveConsts.MountingLocations> kinematics;
	private Map<Constants.SwerveDriveConsts.MountingLocations, SwerveModule> modules = new HashMap<>();
	// private
	// SwerveLimiter.RotationDirectionCorrectorGetter<Constants.SwerveDrive.MountingLocations>
	// directionCorectorGetter;
	private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();
	private double speedFactor = Constants.SwerveDriveConsts.defaultSpeedFactor;

	private void setUpSwerveKinematics() {
		Map<Constants.SwerveDriveConsts.MountingLocations, Translation2d> mountingPoints = Constants.SwerveDriveConsts.swerveModuleConfigs
				.entrySet().stream().map(Algorithms.mapEntryFunction(config -> config.mountingPoint))
				.collect(Collectors.toMap(Entry::getKey, Entry::getValue));
		kinematics = new SwerveKinematics<Constants.SwerveDriveConsts.MountingLocations>(mountingPoints);
	}

	private void setUpSwerveModules() {
		modules = Constants.SwerveDriveConsts.swerveModuleConfigs.entrySet().stream()
				.map(Algorithms.mapEntryFunction(SwerveModule::new))
				.collect(Collectors.toMap(Entry::getKey, Entry::getValue));
		forEachModuleEntry(moduleEntry -> Shuffleboard.getTab("Drive")
				.add("SwerveModule" + moduleEntry.getKey().toString(), moduleEntry.getValue()));
	}

	private SwerveDrive() {
		setUpSwerveModules();
		setUpSwerveKinematics();
		// directionCorectorGetter = Constants.SwerveDrive.directionCorectorGetter;
	}

	public static SwerveDriveBase getInstance() {
		if (instance == null)
			if (Constants.SwerveDriveConsts.enabled) {
				instance = new SwerveDrive();
				instance.setDefaultCommand(new DefaultDriveCommand());
				// if (!Constants.MecanumDrive.IS_ENABLED)
				// throw new Error("Tank drive can't be enabled while swerve drive is anabled");
			} else {
				instance = new SwerveDriveBase();
			}
		return instance;
	}

	@Override
	public boolean isModuleZeroed(Constants.SwerveDriveConsts.MountingLocations mountingLocation) {
		return modules.get(mountingLocation).hasEncoderBeenZeroed();
	}

	@Override
	public void withModule(Constants.SwerveDriveConsts.MountingLocations mountingLocation, Consumer<SwerveModule> consumer) {
		consumer.accept(modules.get(mountingLocation));
	}

	private double getMaxSpeed(Map<Constants.SwerveDriveConsts.MountingLocations, SwerveModuleState> states) {
		return states.values().stream().max(Comparator.comparing(state -> state.speedMetersPerSecond))
				.get().speedMetersPerSecond;
	}

	private Map<Constants.SwerveDriveConsts.MountingLocations, SwerveModuleState> normalizeStates(
			Map<Constants.SwerveDriveConsts.MountingLocations, SwerveModuleState> states) {
		if (getMaxSpeed(states) > Constants.SwerveDriveConsts.maxSpeedOfDrive * speedFactor)
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
		Map<Constants.SwerveDriveConsts.MountingLocations, SwerveModuleState> states = kinematics
				.toLabledSwerveModuleStates(currentChassisSpeeds);
		states = normalizeStates(states);

		states.entrySet().forEach(
				(Entry<Constants.SwerveDriveConsts.MountingLocations, SwerveModuleState> labeledState) -> modules
						.get(labeledState.getKey()).setDesiredState(labeledState.getValue()));

		forEachModule(module -> module.drive(speedFactor));
	}

	@Override
	public void rotateAllModules(double speed) {
		forEachModule(module -> module.rotateModule(speed));
	}

	@Override
	public Map<Constants.SwerveDriveConsts.MountingLocations, Boolean> areHalSensoredOfMoudlesTriggered() {
		Map<Constants.SwerveDriveConsts.MountingLocations, Boolean> result = new HashMap<>();
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
		return joystickValue * Constants.SwerveDriveConsts.maxSpeedOfDrive;
	}

	public static double joystickInputToRadPerSecond(double joystickValue) {
		return joystickValue * Constants.SwerveDriveConsts.maxRotationSpeed;
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
		builder.addDoubleProperty("odometry x",() -> Swervdrive_poseestimator.getInstance().swerveDrivePoseEstimator.getEstimatedPosition().getX(), null);
		builder.addDoubleProperty("odometry y",() -> Swervdrive_poseestimator.getInstance().swerveDrivePoseEstimator.getEstimatedPosition().getY(), null);
		builder.addDoubleProperty("odometry rot",() -> Swervdrive_poseestimator.getInstance().swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), null);
	}

	public SwerveDriveKinematics getKinematics() {
		return kinematics;
	}

	@Override
	public void forEachModuleEntry(
			Consumer<Map.Entry<Constants.SwerveDriveConsts.MountingLocations, SwerveModule>> consumer) {
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
		ArrayList<SwerveModulePosition> pos = new ArrayList();
		for (var module:modules.values()){
			pos.add(module.getOdometryPos());
		}
		return pos.toArray(SwerveModulePosition[]::new);
	}

	@Override
	public void configureButtonBindings(Joystick joystick) {
	}

	@Override
	public List<Binding> getMappings() {
		var joystick = Constants.Joystick.id;
		return List.of(
				new Binding(joystick, Constants.SwerveDriveConsts.ButtounIds.zeroNavx, Trigger::onTrue,
						new InstantCommand(FridoNavx.getInstance()::reset)),
				new Binding(joystick, Constants.SwerveDriveConsts.ButtounIds.zeroEncoders, Trigger::onTrue,
						new ZeroEncoders()),
				new Binding(joystick, Constants.SwerveDriveConsts.ButtounIds.fullSpeed, Trigger::toggleOnTrue,
						new SetSpeedFactor(Constants.SwerveDriveConsts.fullSpeed)),
				new Binding(joystick, Constants.SwerveDriveConsts.ButtounIds.slowSpeed, Trigger::toggleOnTrue,
						new SetSpeedFactor(Constants.SwerveDriveConsts.slowSpeedFactor)),
				new Binding(joystick, Constants.SwerveDriveConsts.ButtounIds.fieldOriented, Trigger::onTrue,
						new InstantCommand(() -> setDriveMode(DriveOrientation.FieldOriented))),
				new Binding(joystick, Constants.SwerveDriveConsts.ButtounIds.shooterFrontOriented, Trigger::onTrue,
						new InstantCommand(() -> setDriveMode(DriveOrientation.ShooterFront))),
				new Binding(joystick, Constants.SwerveDriveConsts.ButtounIds.shooterBackOriented, Trigger::onTrue,
						new InstantCommand(() -> setDriveMode(DriveOrientation.ShooterBack))),
				new Binding(joystick, Constants.SwerveDriveConsts.ButtounIds.zeorOdometry, Trigger::onTrue,
						new InstantCommand(Swervdrive_poseestimator.getInstance()::reset_odometry)),
				new Binding(joystick, Constants.SwerveDriveConsts.ButtounIds.startcom, Trigger::onTrue,
						new InstantCommand(SwervdriveAuto.getInstance()::startCommand)));
	}
}
