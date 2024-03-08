
package frc.robot.abstraction.baseClasses;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.module.Module;
import frc.robot.Config;
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

	@Override
	public double percent2rotationVelocityDouble(double val) {
		// Attention: For swerves the turning speed of the single wheels?
		return Config.data().auto().maxTurnSpeed().in(RadiansPerSecond) * val;
	}

	@Override
	public Measure<Velocity<Angle>> percent2rotationVelocity(double val) {
		// Attention: For swerves the turning speed of the single wheels?
		return Config.data().auto().maxTurnSpeed().times(val);
	}

	@Override
	public Measure<Velocity<Distance>> percent2driveVelocity(double x) {
		return Config.data().auto().maxVelocity().times(x);
	}

}

