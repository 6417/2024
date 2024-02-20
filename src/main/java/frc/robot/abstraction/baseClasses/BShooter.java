package frc.robot.abstraction.baseClasses;

import java.util.List;

import frc.fridowpi.module.Module;
import frc.robot.abstraction.interfaces.IShooter;

/**
 * BShooter: Base class for all shooter subsystems
 */
public abstract class BShooter extends Module implements IShooter {

	public static class ShooterData {
		public final List<Integer> motorIds;

		public ShooterData(List<Integer> motorIds) {
			this.motorIds = motorIds;
		}
	}

	abstract public ShooterData getData();
}
