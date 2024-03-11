package frc.robot.abstraction;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
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

	public record HardwareData(
			Measure<Distance> wheelCircumference,
			Measure<Distance> trackWidth,
			double encoderToMeters) {
	}

	public record DriveData(
			boolean enabled,
			List<Integer> motorIds,
			List<MotorRole> inverted) {
	}

	public record AutoData(
			Measure<Velocity<Distance>> maxVelocity,
			Measure<Velocity<Velocity<Distance>>> maxAcceleration,
			Measure<Velocity<Angle>> maxTurnSpeed,
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
						List.of()),
				new AutoData(
						MetersPerSecond.of(0),
						MetersPerSecondPerSecond.of(0),
						RotationsPerSecond.of(0),
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
