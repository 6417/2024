package frc.robot.subsystems;


//-------- Does not work---------//

public class Tankdrive_odometry {

    public static instance;

    private Tankdrive_odometry(){
        DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
        m_gyro.getRotation2d(),m_leftEntcoder.getDistance(), m_rightEncoder.getdistance(),
        new Pose2d(5,13.5, new Roation2d()));
    }

    public void update_robot_pose(){
        var gyroAngle = m_gyro.getRotation2d();

        m_pose = m_odometry.update(gyroAngle,m_leftEntcoder.getDistance(),
        m_rightEncoder.getDistance());
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
