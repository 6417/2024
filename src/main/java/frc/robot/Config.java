package frc.robot;

import frc.fridolib.RobotPresets.RobotPreset;
import frc.robot.interfaces.IShooter;
import frc.robot.interfaces.abstract_base_classes.BDrive;

public class Config {
	public static final RobotPreset activePreset = RobotPreset.TestChassisDrive;
	
	public static BDrive drive() {
		assert activePreset.getDrive().isPresent():
			"No drive found for preset " + activePreset.getClass().getSimpleName();
		return activePreset.getDrive().get();
	}

	public static IShooter shooter() {
		assert activePreset.getShooter() != null:
			"No shooter found for preset " + activePreset.getClass().getSimpleName();
		return activePreset.getShooter().get();
	}
}
