package frc.robot;

import java.util.Map;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.fridowpi.module.Module;
import frc.robot.abstraction.baseClasses.BDrive.SpeedFactor;

/**
 * Holds the data concerning input, which should be available
 * either to the entire program or get exported to the shuffleboard
 */
public class Controls extends Module {
	public static Map<SpeedFactor, Double> speedFactors = Map.of(
			SpeedFactor.DEFAULT_SPEED, 0.8,
			SpeedFactor.FAST, 1.0,
			SpeedFactor.SLOW, 0.4);
	private static SpeedFactor activeSpeedFactor = SpeedFactor.DEFAULT_SPEED;
	private static double deadBandDrive = 0.15;
	private static double deadBandTurn = 0.15;

	private static double accelerationSensitivity = speedFactors.get(activeSpeedFactor);
	private static double turnSensitivity = 0.6;

	public static void setActiveSpeedFactor(SpeedFactor speedFactor) {
		activeSpeedFactor = speedFactor;
	}

	// Getters and setters
	public static double getAccelerationSensitivity() {
		return accelerationSensitivity;
	}

	public static double getTurnSensitivity() {
		return turnSensitivity;
	}

	public static SpeedFactor getActiveSpeedFactor() {
		return activeSpeedFactor;
	}

	// Shuffleboard
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Motor Controller");

		builder.addDoubleProperty("turnSensitivity", () -> turnSensitivity,
				val -> turnSensitivity = val);

		builder.addDoubleProperty("defaultSpeedFactor", () -> speedFactors.get(SpeedFactor.DEFAULT_SPEED),
				val -> speedFactors.put(SpeedFactor.DEFAULT_SPEED, val));
		builder.addDoubleProperty("slowSpeedFactor", () -> speedFactors.get(SpeedFactor.SLOW),
				val -> speedFactors.put(SpeedFactor.SLOW, val));
		builder.addDoubleProperty("fastSpeedFactor", () -> speedFactors.get(SpeedFactor.FAST),
				val -> speedFactors.put(SpeedFactor.FAST, val));
	}

	public static double getDriveDeadband() {
		return deadBandDrive;
	}

	public static double getTurnDeadband() {
		return deadBandTurn;
	}
}
