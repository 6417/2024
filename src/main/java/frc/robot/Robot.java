package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.joystick.Joystick2024;
import frc.robot.joystick.IdsWithState.State;

public class Robot extends TimedRobot {

    @Override
    public void robotInit() {

		FridoNavx.setup(SPI.Port.kMXP);

		Config.active.initAll();

		// Setup joysticks
		Joystick2024.getInstance().setup(State.DEFAULT);

        // Shuffleboard //
        Shuffleboard.getTab("Drive").add(Config.drive());
        Shuffleboard.getTab("Joystick").add("joystick", Joystick2024.getInstance());
        // Shuffleboard.getTab("Shooter").add(shooter);
        Shuffleboard.getTab("Debug").add(CommandScheduler.getInstance());

        SignalLogger.setPath("/home/lvuser/logs");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

}
