package frc.robot;

import java.util.List;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.joystick.Joystick2024;
import frc.robot.joystick.IdsWithState.State;

public class Robot extends TimedRobot {

    @Override
    public void robotInit() {

		FridoNavx.setup(SPI.Port.kMXP);

		// Setup joysticks
        JoystickHandler.getInstance().setupJoysticks(List.of(Constants.Joystick.primaryJoystickId));
        JoystickHandler.getInstance().init();
        JoystickHandler.getInstance().bind(Config.drive());
        JoystickHandler.getInstance().init();

		Config.active.initAll();

        // Shuffleboard //
        Shuffleboard.getTab("Drive").add(Config.drive());
        Shuffleboard.getTab("Joystick").add("joystick", Joystick2024.getInstance());
        // Shuffleboard.getTab("Shooter").add(shooter);
        Shuffleboard.getTab("Debug").add(CommandScheduler.getInstance());

        SignalLogger.setPath("/home/lvuser/logs");

        // Tankdrive_odometry.getInstance().reset_odometry();
        Joystick2024.getInstance().setup(State.SYSID_TUNING);
        Joystick2024.getInstance().setState(State.SYSID_TUNING);
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
