package frc.robot.subsystems.vision_autonomous;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.Constants.Swervedrive;
import frc.robot.subsystems.drive.Tankdrive;
import frc.robot.subsystems.drive.swerve.SwerveDrive;

public class Swervdrive_poseestimator extends SubsystemBase {
  public SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  public static Swervdrive_poseestimator instance;
  Timer timer;

  public Swervdrive_poseestimator() {
    double[] pos = Visionprocessing.getInstance().getFieldPos();
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(SwerveDrive.getInstance().getKinematics(),
        FridoNavx.getInstance().getRotation2d(), SwerveDrive.getInstance().getOdometryPoses(),
        new Pose2d(pos[0], pos[1], new Rotation2d(Units.degreesToRadians(pos[5]))));
    timer = new Timer();
    timer.start();
  }

  private double get_dist_to_apriltag() {
    double[] pos = Visionprocessing.getInstance().getData();
    return Math.sqrt(pos[0] * pos[0] + pos[1] * pos[1]);
  }

  private double stand_div_vision(double x) {
    final double maxDist = 12;
    final double div_at_max_dis = 8.5;
    final double divmin = 0.05;
    return Math.tanh(x / maxDist) * (div_at_max_dis / Math.tanh(1) + divmin);
  }

  public void update() {
    Rotation2d gyroAngle = FridoNavx.getInstance().getRotation2d();
    swerveDrivePoseEstimator.updateWithTime(timer.get(), gyroAngle, SwerveDrive.getInstance().getOdometryPoses());

    int t = Visionprocessing.getInstance().validTarget();

    if (t == 1) {
      double[] visionPosition = Visionprocessing.getInstance().getFieldPos();
      Pose2d pos = new Pose2d(visionPosition[0], visionPosition[1], gyroAngle);
      double dist = get_dist_to_apriltag();
      double standDiv = stand_div_vision(dist);
      // System.out.println(standDiv);
      swerveDrivePoseEstimator.addVisionMeasurement(pos, timer.get(), VecBuilder.fill(standDiv, standDiv, 0.5));
      // Gyro.getInstance().reset();
      // if (visionPosition[5] < 0){
      // visionPosition[5] = visionPosition[5]; //360-
      // }
    }
  }

  public void reset_odometry() {
    swerveDrivePoseEstimator.resetPosition(FridoNavx.getInstance().getRotation2d(), SwerveDrive.getInstance().getOdometryPoses(), new Pose2d(0,0, new Rotation2d(0)));
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }

  public static Swervdrive_poseestimator getInstance() {
    if (instance == null) {
      instance = new Swervdrive_poseestimator();
    }
    return instance;
  }
}
