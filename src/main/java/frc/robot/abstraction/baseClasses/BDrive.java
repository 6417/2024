
package frc.robot.abstraction.baseClasses;

import java.util.List;

import frc.fridowpi.joystick.Binding;
import frc.fridowpi.module.Module;
import frc.robot.abstraction.interfaces.IDrive;

/**
 * BDrive: Abstract base class for all drive subsystems
 * Can't just be a interface because it needs the methods of SybsystemBase
 */
public abstract class BDrive extends Module implements IDrive {

	@Override
	public List<Binding> getMappings() {
		return List.of();
	}

}

