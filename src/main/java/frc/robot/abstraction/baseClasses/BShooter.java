package frc.robot.abstraction.baseClasses;

import edu.wpi.first.util.sendable.Sendable;
import frc.robot.abstraction.interfaces.IShooter;

/**
 * BShooter: Base class for all shooter subsystems
 */
public abstract class BShooter implements IShooter, Sendable {

	@Override
	public abstract void shoot();

}
