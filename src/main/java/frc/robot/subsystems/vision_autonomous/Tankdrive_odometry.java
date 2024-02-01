package frc.robot.subsystems.vision_autonomous;

import static edu.wpi.first.units.Units.Radians;

import org.apache.logging.log4j.core.appender.routing.RoutingAppender;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.robot.subsystems.drive.Tankdrive;

public class Tankdrive_odometry {

    public static Tankdrive_odometry instance;

    public DifferentialDriveOdometry m_odometry;

    private Tankdrive_odometry(){
        m_odometry = new DifferentialDriveOdometry(
        Gyro.getInstance().getRotation2d(),Tankdrive.getInstance().getLeftEncoderPos(), Tankdrive.getInstance().getRightEndocderPos(),
        new Pose2d(5,13.5, new Rotation2d(0)));
    }

    public void update_robot_pose(){
        Rotation2d gyroAngle = Gyro.getInstance().getRotation2d();

        Pose2d m_pose = m_odometry.update(gyroAngle,Tankdrive.getInstance().getLeftEncoderPos(),
        Tankdrive.getInstance().getRightEndocderPos());

        int t = Visionprocessing.getInstance().validTarget();

        if (true && t == 1){
            double[] visionPosition = Visionprocessing.getInstance().getFieldPos();
            //double[] visionPosition = Visionprocessing.getInstance().get_abs_pose_on_field();
            //System.out.println(visionPosition[1]);
            Pose2d pos = new Pose2d(visionPosition[0],visionPosition[1],gyroAngle);
            //m_odometry.update(gyroAngle,visionPosition[0],visionPosition[1]);
            //System.out.println(visionPosition[5]);
            m_odometry.resetPosition(gyroAngle, Tankdrive.getInstance().getWeelPosition(), pos);
            Gyro.getInstance().reset();
            if (visionPosition[5] < 0){
                visionPosition[5] = 360+visionPosition[5];
            }
            Gyro.getInstance().setAngle(-visionPosition[5]);
            //System.out.println(m_odometry.getPoseMeters().getRotation().getDegrees());
            //m_odometry.update(new Rotation2d(visionPosition[5]),visionPosition[0],visionPosition[1]);
        }
    }

    public void reset_odometry(){
        m_odometry.resetPosition(new Rotation2d(0), Tankdrive.getInstance().getWeelPosition(), 
        new Pose2d(0,0, new Rotation2d(0)));
    }

    public static Tankdrive_odometry getInstance(){
        if (instance == null){
            instance = new Tankdrive_odometry();
        }
        return instance;
    }
}
