package frc.robot.abstraction;

import java.util.List;
import java.util.Map;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.*;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.abstraction.baseClasses.BDrive.SpeedFactor;
import frc.robot.subsystems.drive.tankdrive.MotorSet.MotorRole;

/**
 * RobotData: Holds important constants of the active robot.
 */
public record RobotData(
		HardwareData hardware,
		DriveData drive,
		AutoData auto,
		PidData pid) {

	public record HardwareData(
			Measure<Distance> wheelCircumference,
			Measure<Distance> trackWidth,
			double encoderToMeters) {
	}

	public record DriveData(
			boolean enabled,
			List<Integer> motorIds,
			List<MotorRole> inverted,
			Map<SpeedFactor, Double> speedFactors) {
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

	/* Constructors */
	public RobotData() {
		this(new HardwareData(
				Meters.of(0),
				Meters.of(0),
				0),
				new DriveData(
						false,
						List.of(),
						List.of(),
						Map.of()),
				new AutoData(
						MetersPerSecond.of(0),
						MetersPerSecondPerSecond.of(0),
						Meters.of(0),
						MetersPerSecond.of(0),
						MetersPerSecondPerSecond.of(0),
						Meters.of(0),
						Seconds.of(0)),
				new PidData(
						new PidValues(0, 0, 0),
						new PidValues(0, 0, 0),
						new PidValues(0, 0, 0)));
	}
}
