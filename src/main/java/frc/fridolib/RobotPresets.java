package frc.fridolib;

import java.util.Optional;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.swerve.SwerveModulePhoenixSparkMax;
import frc.robot.subsystems.drive.tankdrive.FourFalcons;

public class RobotPresets {

    /* Base class for the presets, declares the subsystems that could be available */
    public static class RobotPreset {
        public Optional<DriveBase> drive() {
            return null;
        }
        // public static final ArmBase Arm = null;
        public Optional<ShooterSubsystem> shooter() {
            return null;
        }
        public Optional<SwerveModulePhoenixSparkMax> single_swerve_module() {
            return null;
        }
    }

    public static class TestChassisDrive extends RobotPreset {
        public static final DriveBase DRIVE = new FourFalcons(-1, -1, -1, -1);

        @Override
        public Optional<DriveBase> drive() {
            return Optional.of(DRIVE);
        }
    }

    public static class TestChassisShooter extends RobotPreset {
        public static final DriveBase DRIVE = new FourFalcons(-1, -1, -1, -1);
        public static final ShooterSubsystem SHOOTER = null;
    }

    public static class Demogorgon extends RobotPreset {
    }

    public static class Diplodocus extends RobotPreset {
        public static final DriveBase DRIVE = new FourFalcons(-1, -1, -1, -1);
        // public static final ArmBase ARM = new TwoAxisArm(-1, -1, -1, -1);
    }

    public static class DiplodocusSwerveModule extends RobotPreset {
        public static final DriveBase DRIVE = new FourFalcons(-1, -1, -1, -1);
        public static final SwerveModulePhoenixSparkMax SINGLE_SWERVE_MODULE = new SwerveModulePhoenixSparkMax(1, 3, 0);
    }
}
