package frc.robot;

public class Constants {
    public static final class Testchassi{
        final public static int idRigthfront = 13;
        final public static int idLeftfront = 12;
        final public static int idRigthback = 11;
        final public static int idLeftback = 0;

        public static final class Odometry {
            public static final double wheelPerimeter = 0.47;
            // The transmission denotes how many revolution the motor makes compared to the wheel
            public static final double transmission = 10.71;
            public static final int encoderResolution = 2048;

            public static final double encoderToMetersConversion = 1 / ((1 / wheelPerimeter) * transmission * encoderResolution);
            public static final double trackWidthMeters = 0.5;
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


        //final public static double kMaxVMetersPerSecond = 0;
        //final public static double kMaxAccMetersPerSecond = 0;
        //final public static DifferentialDriveKinematics kDriveKinematics;

        final public static double ksVolts = 0;
        final public static double kvVoltSevondsPerMeter = 0;
        final public static double kaVoltSecondsSquaredPerMeter = 0;
        final public static double kPDriveVel = 0;
    }

    public static final class Tankdrive {

    }

    public static final class Swervedrive {
        public static final class Drive {
            public static final double kMaxSpeedMetersPerSecond = 0;
            public static final double kMaxAccelerationMetersPerSecondSquared = 0;
        }
    }

    public static final class Global {
        public static final int idShooterMotor = 0;
    }
}
