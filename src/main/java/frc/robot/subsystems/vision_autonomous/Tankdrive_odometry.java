package frc.robot.subsystems.vision_autonomous;

import static edu.wpi.first.units.Units.Radians;

import org.apache.logging.log4j.core.appender.routing.RoutingAppender;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.subsystems.drive.Tankdrive;

public class Tankdrive_odometry extends SubsystemBase{

    public static Tankdrive_odometry instance;

    public DifferentialDrivePoseEstimator m_odometry;
    private Timer timer; 

    private Tankdrive_odometry(){
        double[] pos = Visionprocessing.getInstance().getFieldPos();
        m_odometry = new DifferentialDrivePoseEstimator(Tankdrive.getInstance().m_kinematics, FridoNavx.getInstance().getRotation2d(),
        Tankdrive.getInstance().getLeftEncoderPos(), Tankdrive.getInstance().getRightEndocderPos(), new Pose2d(pos[0],pos[1],new Rotation2d(Units.rotationsToDegrees(pos[5]))));
        //attention vision rotation in radians not in degrees
        timer = new Timer();
        timer.start();
    }

    private double get_dist_to_apriltag(){
        double[] pos = Visionprocessing.getInstance().getData();
        return Math.sqrt(pos[0]*pos[0] + pos[1]*pos[1]);
    }

    private double stand_div_vision(double x){
        final double maxDist = 12;
        final double div_at_max_dis = 1.5;
        final double divmin = 0.05;
        return Math.tanh(x/maxDist)*(div_at_max_dis/Math.tanh(1)+divmin);
    }

    public void update_robot_pose(){
        Rotation2d gyroAngle = FridoNavx.getInstance().getRotation2d();

        Pose2d m_pose = m_odometry.updateWithTime(timer.get(),gyroAngle,Tankdrive.getInstance().getLeftEncoderPos(),
        Tankdrive.getInstance().getRightEndocderPos());

        int t = Visionprocessing.getInstance().validTarget();

        if (t == 1){
            double[] visionPosition = Visionprocessing.getInstance().getFieldPos();
            Pose2d pos = new Pose2d(visionPosition[0],visionPosition[1],new Rotation2d(Units.degreesToRadians(visionPosition[5]))); //gyroangle
            double dist = get_dist_to_apriltag();
            double standDiv = stand_div_vision(dist);
            //System.out.println(standDiv);
            m_odometry.addVisionMeasurement(pos, timer.get(), VecBuilder.fill(standDiv,standDiv,0.5));
            //Gyro.getInstance().reset();
            // if (visionPosition[5] < 0){
            //     visionPosition[5] = visionPosition[5]; //360-
            // }
        }
    }

    public void reset_odometry(){
        m_odometry.resetPosition(new Rotation2d(0), Tankdrive.getInstance().getWeelPosition(), 
        new Pose2d(0,0, new Rotation2d(0)));
    }

    @Override
    public void periodic(){
        update_robot_pose();
    }


    public static Tankdrive_odometry getInstance(){
        if (instance == null){
            instance = new Tankdrive_odometry();
        }
        return instance;
    }
}
