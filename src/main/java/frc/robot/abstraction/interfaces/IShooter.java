package frc.robot.abstraction.interfaces;

/**
 * IShooter: Interface for all shooters
 */
public interface IShooter {

	// Executes one shooting cycle, if defined
	public void shoot();

	// Sets the motor speed of the shooter
	public void setSpeedPercent(double speed);

	// Must be called in a .periodic() function, so that the motors are fed
	// If enabled is false, don't run
	public void run(boolean enabled);


	// GETTERS

	public double getSpeedPercent();
	
}
