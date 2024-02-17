package frc.robot.abstraction.baseClasses;

import frc.robot.abstraction.interfaces.IMotorSet;
import frc.fridowpi.module.Module;

/**
 * BMotorSet
 */
public abstract class BMotorSet extends Module implements IMotorSet {

	public BMotorSet(int... motorIds) { }
}
