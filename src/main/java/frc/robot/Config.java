package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.fridolib.RobotPreset;
import frc.robot.abstraction.RobotData;
import frc.robot.abstraction.baseClasses.BDrive;

public class Config {
	public static final RobotPreset active = RobotPreset.TestChassisDrive;
	public static final XboxController joystick = Controls.joystick; // Temporarily
	
	// Drive should always exist
	public static BDrive drive() {
		assert active.getDrive().isPresent():
			"No drive found for preset " + active.getClass().getSimpleName();

		return active.getDrive().get();
	}

	public static RobotData data() {
		return active.getData();
	}
}
