package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.abstraction.baseClasses.BDrive.DriveOrientation;
import frc.robot.joystick.IdsWithState.State;
import frc.robot.joystick.Joystick2024;

public class Robot extends TimedRobot {

    // Aliases for often used singleton instances
    ShuffleboardTab tab;

    @Override
    public void robotInit() {

		FridoNavx.setup(SPI.Port.kMXP);
		FridoNavx.setPitchOffset(90);

		Config.active.initAll();

		// Setup joysticks
		Joystick2024.getInstance().setup(State.DEFAULT);

        // Shuffleboard //
        Shuffleboard.getTab("Drive").add(Config.drive());
        Shuffleboard.getTab("Drive").add(Config.drive().getDefaultCommand());
        Shuffleboard.getTab("Joystick").add("joystick", Joystick2024.getInstance());
        Config.active.getShooter().ifPresent(shooter -> Shuffleboard.getTab("Shooter").add(shooter));
		Config.active.getAuto().ifPresent(auto -> Shuffleboard.getTab("Auto").add(auto));
        Shuffleboard.getTab("Debug").add(CommandScheduler.getInstance());

        SignalLogger.setPath("/home/lvuser/logs");

		Config.drive().setOrientation(DriveOrientation.FieldOriented);
    }

	double maxSpeed = 0;

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

	@Override
	public void teleopInit() {
		Config.drive().setIdleMode(IdleMode.kBrake);
	}

	@Override
	public void autonomousInit() {
		Config.drive().setIdleMode(IdleMode.kBrake);
	}

	@Override
	public void disabledInit() {
		Config.drive().setIdleMode(IdleMode.kCoast);
	}
}
