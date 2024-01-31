package frc.robot.subsystems.vision_autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.robot.subsystems.drive.tankdrive.FourFalcons;

//-------- Does not work---------//

public class Tankdrive_odometry {

    public static Tankdrive_odometry instance;

    DifferentialDriveOdometry m_odometry;

    private Tankdrive_odometry(){
        m_odometry = new DifferentialDriveOdometry(
        Gyro.getInstance().getRotation2d(),FourFalcons.getInstance().getLeftEncoderPos(), FourFalcons.getInstance().getRigthEncoderPos(),
        new Pose2d(5,13.5, new Rotation2d(0)));
    }

    public void update_robot_pose(){
        var gyroAngle = Gyro.getInstance().getRotation2d();

        Pose2d m_pose = m_odometry.update(gyroAngle,FourFalcons.getInstance().getLeftEncoderPos(),
        FourFalcons.getInstance().getRigthEncoderPos());
    }

    public void reset_robot_pose(){
        //m_odometry.resetPosition();
    }

    public static Tankdrive_odometry getInstance(){
        if (instance == null){
            instance = new Tankdrive_odometry();
        }
        return instance;
    }
}
