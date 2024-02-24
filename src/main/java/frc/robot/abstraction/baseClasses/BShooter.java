package frc.robot.abstraction.baseClasses;

import java.util.List;

import frc.fridowpi.module.Module;
import frc.robot.abstraction.interfaces.IShooter;

/**
 * BShooter: Base class for all shooter subsystems
 */
public abstract class BShooter extends Module implements IShooter {

	@Override
	public void init() {
		super.init();
	}

	public static class ShooterData {
		public final List<Double> speeds;
		public final List<Integer> motorIds;

		public ShooterData(List<Integer> motorIds, List<Double> speeds) {
			this.motorIds = motorIds;
			this.speeds = speeds;
		}
	}

	abstract public ShooterData getData();
}
