package frc.robot.subsystems.drive.swerve_2024;

import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.sensors.FridoNavx;
import frc.fridowpi.utils.Algorithms;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BSwerveDrive;
import frc.robot.abstraction.baseClasses.BSwerveModule;
import frc.robot.commands.drive.commands_2024.DriveCommand2024;
import frc.robot.commands.drive.commands_2024.SetSpeedFactor;

// TODO: use velocity PIDs
public class SwerveDrive2024 extends BSwerveDrive {

	private SwerveKinematics<MountingLocations> kinematics;
	private Map<MountingLocations, BSwerveModule> modules = new HashMap<>();
	private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();
	private double speedFactor = Constants.SwerveDrive.Swerve2024.defaultSpeedFactor;

	public SwerveDrive2024() {
	}

	@Override
	public void init() {
		super.init();
		setUpSwerveModules();
		setUpSwerveKinematics();
		zeroEncoders();
		setDefaultCommand(new DriveCommand2024());
	}

	private void zeroEncoders() {
		forEachModuleEntry(moduleEntry -> moduleEntry.getValue().zeroRelativeEncoder());
	}

	private void setUpSwerveKinematics() {
		Map<MountingLocations, Translation2d> mountingPoints = Constants.SwerveDrive.Swerve2024.swerveModuleConfigs
				.entrySet().stream().map(Algorithms.mapEntryFunction(config -> config.mountingPoint))
				.collect(Collectors.toMap(Entry::getKey, Entry::getValue));
		kinematics = new SwerveKinematics<MountingLocations>(mountingPoints);
	}

	private void setUpSwerveModules() {
		modules = Constants.SwerveDrive.Swerve2024.swerveModuleConfigs.entrySet().stream()
				.map(Algorithms.mapEntryFunction(SwerveModule::new))
				.collect(Collectors.toMap(Entry::getKey, Entry::getValue));
		forEachModuleEntry(moduleEntry -> Shuffleboard.getTab("Drive")
				.add("SwerveModule" + moduleEntry.getKey().toString(), moduleEntry.getValue()));
	}

	@Override
	public boolean isModuleZeroed(MountingLocations mountingLocation) {
		return true;
	}

	@Override
	public void withModule(MountingLocations mountingLocation, Consumer<BSwerveModule> consumer) {
		consumer.accept(modules.get(mountingLocation));
	}

	private double getMaxSpeed(Map<MountingLocations, SwerveModuleState> states) {
		return states.values().stream().max(Comparator.comparing(state -> state.speedMetersPerSecond))
				.get().speedMetersPerSecond;
	}

	private Map<MountingLocations, SwerveModuleState> normalizeStates(
			Map<MountingLocations, SwerveModuleState> states) {
		if (getMaxSpeed(states) > Constants.SwerveDrive.Swerve2024.maxSpeedOfDrive * speedFactor)
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

		forEachModule(module -> module.driveForward(speedFactor));
	}

	@Override
	public void rotateAllModules(double speed) {
		forEachModule(module -> module.rotate(speed));
	}

	@Override
	public void setRotationToHome(MountingLocations moduleLocation) {
		modules.get(moduleLocation).setCurrentRotationToEncoderHome();
	}

	public static double joystickInputToMetersPerSecond(double joystickValue) {
		return joystickValue * Constants.SwerveDrive.Swerve2024.maxSpeedOfDrive;
	}

	public static double joystickInputToRadPerSecond(double joystickValue) {
		return joystickValue * Constants.SwerveDrive.Swerve2024.maxRotationSpeed;
	}

	@Override
	public void forEachModule(Consumer<BSwerveModule> consumer) {
		modules.values().stream().forEach(consumer);
	}

	@Override
	public void stopAllMotors() {
		forEachModule(module -> module.stopAllMotors());
	}

	@Override
	public void forEachModuleEntry(
			Consumer<Map.Entry<MountingLocations, BSwerveModule>> consumer) {
		modules.entrySet().stream().forEach(consumer);
	}

	@Override
	public void setSpeedFactor(double speedFactor) {
		assert speedFactor > 0.0 : "speedFactor must be grater than zero";
		this.speedFactor = speedFactor;
	}

	@Override
	public List<Binding> getMappings() {
		var joystick = Constants.Joystick.primaryJoystickId;
		return List.of(
				new Binding(joystick, Constants.SwerveDrive.ButtounIds.zeroNavx, Trigger::onTrue,
						new InstantCommand(FridoNavx.getInstance()::reset)),
				new Binding(joystick, Constants.SwerveDrive.ButtounIds.zeroEncoders, Trigger::onTrue,
						new InstantCommand(this::zeroEncoders)),

				new Binding(joystick, Constants.SwerveDrive.ButtounIds.fullSpeed, Trigger::toggleOnTrue,
						new SetSpeedFactor(Constants.SwerveDrive.Swerve2024.fullSpeed)),
				new Binding(joystick, Constants.SwerveDrive.ButtounIds.slowSpeed, Trigger::toggleOnTrue,
						new SetSpeedFactor(Constants.SwerveDrive.Swerve2024.slowSpeedFactor)),

				new Binding(joystick, Constants.SwerveDrive.ButtounIds.driveFieldOriented, Trigger::onTrue,
						new InstantCommand(() -> setOrientation(DriveOrientation.FieldOriented))),
				new Binding(joystick, Constants.SwerveDrive.ButtounIds.driveForwards, Trigger::onTrue,
						new InstantCommand(() -> setOrientation(DriveOrientation.Forwards))),
				new Binding(joystick, Constants.SwerveDrive.ButtounIds.driveBackwards, Trigger::onTrue,
						new InstantCommand(() -> setOrientation(DriveOrientation.Backwards))));
	}

	@Override
	public void setVolts(double leftvolts, double rigthvolts) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'setVolts'");
	}

	@Override
	public void driveToPos(Pose2d pos) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'driveToPos'");
	}

	@Override
	public void setIdleMode(IdleMode mode) {
		forEachModule(m -> m.setIdleMode(mode));
	}

	@Override
	public Command sysIdQuasistatic(Direction direction) {
		throw new UnsupportedOperationException("Unimplemented method 'sysIdQuasistatic'");
	}

	@Override
	public Command sysIdDynamic(Direction direction) {
		throw new UnsupportedOperationException("Unimplemented method 'sysIdDynamic'");
	}

	// TODO: abstraction over motorrole
	@Override
	public <T> FridolinsMotor getMotor(T motor) {
		throw new UnsupportedOperationException("Unimplemented method 'getMotor'");
	}

	// TODO: Laurin
	@Override
	public Pose2d getPos() {
		throw new UnsupportedOperationException("Unimplemented method 'getPos'");
	}

	// TODO: Is it needed for Swerve?
	@Override
	public double getLeftEncoderPos() {
		throw new UnsupportedOperationException("Unimplemented method 'getLeftEncoderPos'");
	}

	@Override
	public double getRightEncoderPos() {
		throw new UnsupportedOperationException("Unimplemented method 'getRightEncoderPos'");
	}
}
