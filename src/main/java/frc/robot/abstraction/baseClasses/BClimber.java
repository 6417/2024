
package frc.robot.abstraction.baseClasses;

import java.util.List;

import frc.robot.abstraction.interfaces.IClimber;
import frc.fridowpi.module.Module;

/**
 * BRopeClimber
 */
public abstract class BClimber extends Module implements IClimber {

	public class ClimberData {
		public boolean enabled;
		public final List<Integer> motorIds;

		public ClimberData(boolean enabled, List<Integer> motorIds) {
			this.enabled = enabled;
			this.motorIds = motorIds;
		}
	}

	abstract public ClimberData getData();

}
