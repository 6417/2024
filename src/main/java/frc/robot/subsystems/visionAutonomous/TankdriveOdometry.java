package frc.robot.subsystems.visionAutonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.robot.Config;

//-------- Does not work---------//

public class TankdriveOdometry {

    public static TankdriveOdometry instance;

    DifferentialDriveOdometry m_odometry;

    private TankdriveOdometry(){
        m_odometry = new DifferentialDriveOdometry(
        Gyro.getInstance().getRotation2d(), Config.drive().getLeftEncoderPos(), Config.drive().getRightEncoderPos(),
        new Pose2d(5,13.5, new Rotation2d(0)));
    }

    public void update_robot_pose(){
        var gyroAngle = Gyro.getInstance().getRotation2d();

        Pose2d m_pose = m_odometry.update(gyroAngle, Config.drive().getLeftEncoderPos(),
		Config.drive().getRightEncoderPos());
    }

    public void reset_robot_pose(){
        //m_odometry.resetPosition();
    }

    public static TankdriveOdometry getInstance(){
        if (instance == null){
            instance = new TankdriveOdometry();
        }
        return instance;
    }
}
