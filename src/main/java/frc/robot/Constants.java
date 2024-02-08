package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.abstraction.RobotData;

public class Constants {
	public static final class Testchassi {
		final public static int idRigthfront = 13;
		final public static int idLeftfront = 12;
		final public static int idRigthback = 11;
		final public static int idLeftback = 0;

		public static final class Odometry {
			public static final double wheelPerimeterMeters = 0.47;
			public static final double trackWidthMeters = 0.5;
			// The transmission denotes how many revolution the motor makes compared to the
			// wheel
			public static final double transmission = 10.71;
			public static final int encoderResolution = 2048;

			public static final double encoderToMetersConversion = 1
					/ ((1 / wheelPerimeterMeters) * transmission * encoderResolution);
		}

		public static final class PathWeaver {
			public static final double ksMeters = 0.12091;
			public static final double kvMetersPerSecoond = 2.3501;
			public static final double ka = 0.21997;

			public static final double kMaxVMetersPerSecond = 3.3;
			public static final double kMaxAccMetersPerSecond = 1.2;
			public static final double kMaxCentripetalAcceleration = 0;

			public static final double kRamsetB = 0;
			public static final double kRamseteZeta = 0;

			public static final double kP = 0.36205;
			public static final double kI = 0;
			public static final double kD = 0;

		}

		final public static double ksVolts = 0.20; // random value
		final public static double kvVoltSevondsPerMeter = 1.98; // random value
		final public static double kaVoltSecondsSquaredPerMeter = 0.2; // random value
		final public static double kPDriveVel = 5; // random value

		RobotData data = new RobotData(
				Meters.of(Odometry.wheelPerimeterMeters),
				Meters.of(Odometry.trackWidthMeters),
				Odometry.encoderToMetersConversion,
				Meters.of(PathWeaver.ksMeters),
				MetersPerSecond.of(PathWeaver.kvMetersPerSecoond),
				MetersPerSecondPerSecond.of(PathWeaver.ka));
	}

	public static final class Swervedrive {
		public static final class Drive {
			public static final double kMaxSpeed = 0;
			public static final double kMaxAcceleration = 0;

			public static final double gearRatio = 1.0 / 5.192308;
		}

		// ATTENTION: Referes to the data of TestChassis, needs to be updated as soon as possible
		public RobotData data = new RobotData(
				Meters.of(Testchassi.Odometry.wheelPerimeterMeters),
				Meters.of(Testchassi.Odometry.trackWidthMeters),
				Testchassi.Odometry.encoderToMetersConversion,
				Meters.of(Testchassi.PathWeaver.ksMeters),
				MetersPerSecond.of(Testchassi.PathWeaver.kvMetersPerSecoond),
				MetersPerSecondPerSecond.of(Testchassi.PathWeaver.ka));
	}

	public static final class Global {
		public static final int idShooterMotor = 0;
	}

	public static final class Shooter {
		public static final double OptimalAmpSpeed = 0.3;
		public static final double OptimalSpeakerSpeed = 0.9;
		public static final double OptimalIntakeSpeed = -0.6;
	}

	public static final class Sysid {
		public static final boolean isTuning = false;
	}
}
