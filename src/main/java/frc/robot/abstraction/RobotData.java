package frc.robot.abstraction;

import java.util.List;

import edu.wpi.first.units.*;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.subsystems.drive.tankdrive.MotorSet.MotorRole;

/**
 * RobotData: Holds important constants of the active robot.
 */
public record RobotData(
		HardwareData hardware,
		DriveData drive,
		AutoData auto,
		PidData pid) {

	public record DriveData(
			boolean enabled,
			List<Integer> motorIds,
			List<MotorRole> inverted) {
	}

	public record HardwareData(
			Measure<Distance> wheelCircumference,
			Measure<Distance> trackWidth,
			double encoderToMeters) {
	}

	public record AutoData(
			Measure<Velocity<Distance>> maxVelocity,
			Measure<Velocity<Velocity<Distance>>> maxAcceleration,
			Measure<Distance> ksVolts,
			Measure<Velocity<Distance>> kvVolts,
			Measure<Velocity<Velocity<Distance>>> kaVolts,
			Measure<Distance> kRamseteB,
			Measure<Time> kRamseteZeta) {
	}

	public record PidData(
			PidValues pathWeaver,
			PidValues driveLeft,
			PidValues driveRight) {
	}
}
