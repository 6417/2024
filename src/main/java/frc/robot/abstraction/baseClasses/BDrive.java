
package frc.robot.abstraction.baseClasses;

import java.util.List;

import frc.fridowpi.joystick.Binding;
import frc.fridowpi.module.Module;
import frc.robot.abstraction.interfaces.IDrive;

/**
 * BDrive: Abstract base class for all drive subsystems
 **/
public abstract class BDrive extends Module implements IDrive {

	public enum DriveOrientation {
		FieldOriented, Forwards, Backwards
	}

	public enum MountingLocations {
		FrontRight, FrontLeft, BackRight, BackLeft
	}

	public enum SpeedFactor {
		DEFAULT_SPEED, SLOW, FAST;
	}

	protected DriveOrientation driveOrientation = DriveOrientation.FieldOriented;

	@Override
	public final DriveOrientation getOrientation() {
		return driveOrientation;
	}

	@Override
	public void setOrientation(DriveOrientation driveMode) {
		this.driveOrientation = driveMode;
	}

	@Override
	public List<Binding> getMappings() {
		return List.of();
	}

	@Override
	public void init() {
		super.init();
	}

}

