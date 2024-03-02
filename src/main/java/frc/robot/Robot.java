package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.joystick.IdsWithState.State;
import frc.robot.joystick.Joystick2024;
import frc.robot.subsystems.drive.PidTuner;
import frc.robot.abstraction.baseClasses.BDrive.DriveOrientation;

public class Robot extends TimedRobot {

    // Aliases for often used singleton instances
    ShuffleboardTab tab;
    PidTuner pidtuner = new PidTuner();

    @Override
    public void robotInit() {

		FridoNavx.setup(SPI.Port.kMXP);

		Config.active.initAll();

		// Setup joysticks
		Joystick2024.getInstance().setup(State.DEFAULT);

        // Shuffleboard //
        Shuffleboard.getTab("Drive").add(Config.drive());
        Shuffleboard.getTab("Drive").add(Config.drive().getDefaultCommand());
        Shuffleboard.getTab("Joystick").add("joystick", Joystick2024.getInstance());
        Config.active.getShooter().ifPresent(shooter -> Shuffleboard.getTab("Shooter").add(shooter));
        Shuffleboard.getTab("Debug").add(CommandScheduler.getInstance());

        SignalLogger.setPath("/home/lvuser/logs");

		Config.drive().setOrientation(DriveOrientation.FieldOriented);
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
        //exampleServo.setBoundsMicroseconds(2150, 1501, 1500, 1499, 850);
        //exampleServo.setPulseTimeMicroseconds((int) (000*Controls.joystick.getRightX()));
        //exampleServo.set(Controls.joystick.getRightX());

        //bindButtonsForSysid();
    }
}
