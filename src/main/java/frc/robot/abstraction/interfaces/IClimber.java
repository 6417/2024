package frc.robot.abstraction.interfaces;

import frc.fridowpi.module.IModule;

/**
 * IRopeClimber
 */
public interface IClimber extends IModule {

	public void release();

	public void retract();

	// Must be called periodically
	public void run();

	public void oneStepDown(double speed);

	public void stopMotors();
}
