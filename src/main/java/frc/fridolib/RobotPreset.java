package frc.fridolib;

import java.util.Optional;

import frc.robot.Constants;
import frc.robot.abstraction.RobotData;
import frc.robot.abstraction.baseClasses.BDrive;
import frc.robot.abstraction.baseClasses.BShooter;
import frc.robot.abstraction.interfaces.ISwerveModule;
// import frc.robot.subsystems.drive.swerve_2024.SwerveModulePhoenixSparkMax;
import frc.robot.subsystems.drive.tankdrive.FourFalconsTankDrive;

/** Preset robot configurations
 *
 * Allows for easy switching of robot configurations. For any configuration that is implemented
 * or being developed, a preset can be added to the RobotPreset enum.
 *
 * Note:
 *	- If a subsystem is added, it should have an interface (or baseclass) that is as generic as possible
 *  - A matching constructor must be defined as well (if none is matching, write your own)
 *	- It might be necessary to add more <public Optional> variables and getters for the subsystems
 *
 *	- If possible, take whatever is already there, and don't be afraid to modify it:
 *		- For drives, extend BDrive -- all functions from IDrive and BDrive will be known
 *		- For shooters, extend BShooter, etc...
 *
 * Note to me:
 *	- I should add the joystick configuration here as well
 *	- If someone reads this after 2024 and the above comment is still there, please remind me (@lrshsl)
 *		- (Or add it yourself)
 *
 */
public enum RobotPreset {

	// Enum variants //

	Diplodocus(Constants.Diplodocus.robotData, new FourFalconsTankDrive()),
	TestChassisDrive(Constants.Testchassis.robotData, new FourFalconsTankDrive()),
	// TestChassisShooter(new FourFalconTankDrive(-1, -1, -1, -1), new ShooterTwoPhoenix(-1, -1)),
	// Demogorgon(new TalonSRXSwerveDrive(-1, -1, -1, -1))
	// DiplodocusSwerveModule(new FourFalconsTankDrive(-1, -1, -1, -1), new SwerveModulePhoenixSparkMax(1, 3, 0)),
	;


	// Variables //

	public Optional<BDrive> DRIVE = Optional.empty();
	// public final Optional<IArm> ARM = null;
	public Optional<BShooter> SHOOTER = Optional.empty();
	public Optional<ISwerveModule> SINGLE_SWERVE_MODULE = Optional.empty();
	public RobotData ROBOT_DATA = null;

	// Functions //
	public void initAll() {
		if (DRIVE.isPresent()) { DRIVE.get().init(); }
		if (SHOOTER.isPresent()) { SHOOTER.get().init(); }
		if (SINGLE_SWERVE_MODULE.isPresent()) {
			SINGLE_SWERVE_MODULE.get().init();
		}
	}

	// Getters / Accessors //

	public Optional<BDrive> getDrive() {
		return DRIVE;
	}

	public Optional<BShooter> getShooter() {
		return SHOOTER;
	}

	public RobotData getData() {
		return ROBOT_DATA;
	}

	// Constructors //

	private RobotPreset(RobotData data, BDrive drive) {
		ROBOT_DATA = data;
		DRIVE = Optional.of(drive);
	}

	private RobotPreset(RobotData data, BDrive drive, ISwerveModule swerveModule) {
		ROBOT_DATA = data;
		DRIVE = Optional.of(drive);
		SINGLE_SWERVE_MODULE = Optional.of(swerveModule);
	}

	private RobotPreset(RobotData data, BDrive drive, BShooter shooter) {
		ROBOT_DATA = data;
		DRIVE = Optional.of(drive);
		SHOOTER = Optional.of(shooter);
	}
}
