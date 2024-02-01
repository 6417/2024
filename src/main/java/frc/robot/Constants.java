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
            //public static final double ksMeters = 0.12091;
            //public static final double kvMetersPerSecoond = 2.3501;
            //public static final double ka = 0.21997;

            public static final double kMaxVMetersPerSecond = 1;
            public static final double kMaxAccMetersPerSecond = 0.9;
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

        final public static double ksVolts = 0.40; //random value
        final public static double kvVoltSevondsPerMeter = 1.98; //random value
        final public static double kaVoltSecondsSquaredPerMeter = 0.4; //random value
        final public static double kPDriveVel = 0.1; //random value

        final public static double ticsToMeter = 0.046;
    }

    public static final class Tankdrive {

    }

    public static final class Swervedrive {
        public static final class Drive {
            public static final double kMaxSpeedMetersPerSecond = 0;
            public static final double kMaxAccelerationMetersPerSecondSquared = 0;

            public static final double gearRatio = 1.0 / 5.192308;
        }
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
