package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.SPI.Port;

public class Constants {
    public static class Testchassi{
        final public static int idRigthfront = 13;
        final public static int idLeftfront = 12;
        final public static int idRigthback = 11;
        final public static int idLeftback = 0;

        final public static double kMaxVMetersPerSecond = 0;
        final public static double kMaxAccMetersPerSecond = 0;
        //final public static DifferentialDriveKinematics kDriveKinematics;

        final public static double ksVolts = 0;
        final public static double kvVoltSevondsPerMeter = 0;
        final public static double kaVoltSecondsSquaredPerMeter = 0;
        final public static double kPDriveVel = 0;

        final public static double kRamsetB = 0;
        final public static double kRamseteZeta = 0;
    }
    
    final public static double TalonFX_endcoders_to_meters = 1;
}
