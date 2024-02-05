package frc.fridolib;

import java.util.Optional;

import frc.robot.interfaces.IShooter;
import frc.robot.interfaces.ISwerveModule;
import frc.robot.interfaces.abstract_base_classes.BDrive;
import frc.robot.subsystems.drive.swerve.SwerveModulePhoenixSparkMax;
import frc.robot.subsystems.drive.tankdrive.FourFalconsTankDrive;

public class RobotPresets {
	public static enum RobotPreset {
		Diplodocus(new FourFalconsTankDrive(-1, -1, -1, -1)),
		TestChassisDrive(new FourFalconsTankDrive(-1, -1, -1, -1)),
		// TestChassisShooter(new FourFalconTankDrive(-1, -1, -1, -1), new ShooterTwoPhoenix(-1, -1)),
		// Demogorgon(new TalonSRXSwerveDrive(-1, -1, -1, -1), null)
		DiplodocusSwerveModule(new FourFalconsTankDrive(-1, -1, -1, -1), new SwerveModulePhoenixSparkMax(1, 3, 0)),
		;

		public Optional<BDrive> DRIVE = null;
		// public final Optional<IArm> ARM = null;
		public Optional<IShooter> SHOOTER = null;
		public Optional<ISwerveModule> SINGLE_SWERVE_MODULE = null;

		private RobotPreset(BDrive drive) {
			DRIVE = Optional.of(drive);
		}

		private RobotPreset(BDrive drive, ISwerveModule swerveModule) {
			DRIVE = Optional.of(drive);
			SINGLE_SWERVE_MODULE = Optional.of(swerveModule);
		}

		private RobotPreset(BDrive drive, IShooter shooter) {
			DRIVE = Optional.of(drive);
			SHOOTER = Optional.of(shooter);
		}

        public Optional<BDrive> getDrive() {
            return DRIVE;
        }

        public Optional<IShooter> getShooter() {
            return SHOOTER;
        }
	}
}
