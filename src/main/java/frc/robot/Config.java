package frc.robot;

import frc.fridolib.RobotPreset;
import frc.fridowpi.joystick.IJoystick;
import frc.robot.abstraction.RobotData;
import frc.robot.abstraction.baseClasses.BDrive;
import frc.robot.joystick.Joystick2024;

public class Config {
	public static final RobotPreset active = RobotPreset.ShooterTester;
	
	// Drive should always exist
	public static BDrive drive() {
		assert active.getDrive().isPresent():
			"No drive found for preset " + active.getClass().getSimpleName();

		return active.getDrive().get();
	}

	public static IJoystick joystick() {
		return Joystick2024.getInstance().getPrimaryJoystick();
	}

	public static RobotData data() {
		return active.getData();
	}
}
