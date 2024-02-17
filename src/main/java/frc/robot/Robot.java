package frc.robot;

import java.util.List;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.sensors.FridoNavx;

public class Robot extends TimedRobot {

    // Aliases for often used singleton instances
    // ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    
    @Override
    public void robotInit() {

		FridoNavx.setup(SPI.Port.kMXP);

		// Setup joysticks
        JoystickHandler.getInstance().setupJoysticks(List.of(Constants.Joystick.id));
        JoystickHandler.getInstance().init();
        JoystickHandler.getInstance().bind(Config.drive());
        JoystickHandler.getInstance().init();

		Config.active.initAll();

        // Shuffleboard //
        Shuffleboard.getTab("Drive").add(Config.drive());
        // Shuffleboard.getTab("Shooter").add(shooter);

        // Shuffleboard.getTab("Controls").add(Controls.getInstance()); // wtf nr. 2
        // Shuffleboard.getTab("Debug").add(drive.getDefaultCommand());
        Shuffleboard.getTab("Debug").add(CommandScheduler.getInstance());

        // SignalLogger.setPath("/media/sda1");
        SignalLogger.setPath("/home/lvuser/logs");

        // Tankdrive_odometry.getInstance().reset_odometry();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // Tankdrive_poseestimator.getInstance().updatePoseEstimator();
        // Tankdrive_odometry.getInstance().update_robot_pose();
    }

    @Override
    public void teleopInit() {
        SignalLogger.setPath("test");
    }

    @Override
    public void teleopPeriodic() {
    }
}
