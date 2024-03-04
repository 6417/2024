
package frc.robot.abstraction.baseClasses;

import java.util.List;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.abstraction.interfaces.IClimber;
import frc.fridowpi.module.Module;

/**
 * BRopeClimber
 */
public abstract class BClimber extends Module implements IClimber {

	public class ClimberData {
		public final List<Integer> motorIds;

		public ClimberData(List<Integer> motorIds) {
			this.motorIds = motorIds;
		}
	}

	abstract public Servo getServo();

	abstract public ClimberData getData();

}
