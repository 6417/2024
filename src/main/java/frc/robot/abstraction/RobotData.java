package frc.robot.abstraction;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/**
 * RobotData: Holds immutable data of the active robot.
 */
public record RobotData(
		Measure<Distance> wheelPerimeter,
		Measure<Distance> trackWidth,
		double encoderToMeters,
		Measure<Distance> ks,
		Measure<Velocity<Distance>> kv,
		Measure<Velocity<Velocity<Distance>>> ka) {
}
