package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.abstraction.baseClasses.BDrive.DriveOrientation;
import frc.robot.joystick.IdsWithState.State;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LED.RGB;
import frc.robot.subsystems.ShooterSubsystem.ShooterConfig;
import frc.robot.subsystems.visionAutonomous.SwervedriveAuto;
import frc.robot.joystick.Joystick2024;

public class Robot extends TimedRobot {

	// Aliases for often used singleton instances
	ShuffleboardTab tab;

	@Override
	public void robotInit() {

		FridoNavx.setup(SPI.Port.kMXP);
		FridoNavx.setYawOffset(Constants.SwerveDrive.navxPitchOffset);

		Config.active.initAll();

		// Setup joysticks
		Joystick2024.getInstance().setup(State.DEFAULT);

		// Shuffleboard //
		Shuffleboard.getTab("Controls").add(Controls.instance);
		Shuffleboard.getTab("Drive").add(Config.drive());
		Shuffleboard.getTab("Drive").add(Config.drive().getDefaultCommand());
		Shuffleboard.getTab("Joystick").add("joystick", Joystick2024.getInstance());
		Config.active.getShooter().ifPresent(shooter -> Shuffleboard.getTab("Shooter").add(shooter));
		Config.active.getAuto().ifPresent(auto -> Shuffleboard.getTab("Auto").add(auto));
		Shuffleboard.getTab("Debug").add(CommandScheduler.getInstance());
		Config.active.getClimber().ifPresent(c -> Shuffleboard.getTab("Climber").add(c));

		SignalLogger.setPath("/home/lvuser/logs");

		Config.drive().setOrientation(DriveOrientation.FieldOriented);
	}

	double maxSpeed = 0;

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	Command autocmd = new InstantCommand(
			() -> ((ShooterSubsystem) Config.active.getShooter().get()).shoot(ShooterConfig.SPEAKER));

	@Override
	public void teleopInit() {
		if (autocmd != null && CommandScheduler.getInstance().isScheduled(autocmd)) {
			autocmd.cancel();
		}
		Config.drive().setIdleMode(IdleMode.kBrake);
	}

	@Override
	public void autonomousInit() {
		autocmd.schedule();
		Config.drive().setIdleMode(IdleMode.kBrake);
	}

	@Override
	public void disabledInit() {
		// Config.drive().setIdleMode(IdleMode.kCoast);
		Config.active.getClimber().ifPresent(c -> c.stopMotors());
	}
}
