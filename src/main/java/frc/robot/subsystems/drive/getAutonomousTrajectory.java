package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision_autonomous.Tankdrive_odometry;

public class getAutonomousTrajectory extends SubsystemBase {

    public static getAutonomousTrajectory instance;

    private TrajectoryConfig getTrajectoryConfig() {
        var voltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.Testchassi.ksVolts,
                        Constants.Testchassi.kvVoltSevondsPerMeter,
                        Constants.Testchassi.kaVoltSecondsSquaredPerMeter),
                Tankdrive.getInstance().m_kinematics, 10);

        TrajectoryConfig config = new TrajectoryConfig(Constants.Testchassi.PathWeaver.kMaxVMetersPerSecond,
                Constants.Testchassi.PathWeaver.kMaxAccMetersPerSecond)
                .setKinematics(Tankdrive.getInstance().m_kinematics)
                .addConstraint(voltageConstraint);

        return config;
    }

    public Trajectory get_raw_trajectory() {
        TrajectoryConfig conf = getTrajectoryConfig();
        List<Translation2d> list_translationd2 = new ArrayList<Translation2d>();
        // list_translationd2.add(new Translation2d(1,0.5));
        // list_translationd2.add(new Translation2d(2,1));

        Trajectory new_trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
                list_translationd2,
                new Pose2d(new Translation2d(1, 0), new Rotation2d(0)), conf);
        return new_trajectory;
    }

    private RamseteCommand getTrajectory() {
        TrajectoryConfig conf = getTrajectoryConfig();

        List<Translation2d> list_translationd2 = new ArrayList<Translation2d>();
        //list_translationd2.add(new Translation2d(1, 0.5));
        // list_translationd2.add(new Translation2d(2,1));

        Trajectory new_trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
                list_translationd2,
                new Pose2d(new Translation2d(2, 0), new Rotation2d(0)), conf);

        Trajectory drive_to_apriltag = TrajectoryGenerator.generateTrajectory(Tankdrive_odometry.getInstance().m_odometry.getPoseMeters(),
        list_translationd2,
        new Pose2d(15.5
        ,5.5, new Rotation2d(0)),conf);

        //16.579342, 5.547868 pose of tag 4

        RamseteCommand command = new RamseteCommand(
                drive_to_apriltag,
                Tankdrive.getInstance()::getPos,
                new RamseteController(Constants.Testchassi.PathWeaver.kRamsetB,
                        Constants.Testchassi.PathWeaver.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.Testchassi.ksVolts,
                        Constants.Testchassi.kvVoltSevondsPerMeter,
                        Constants.Testchassi.kaVoltSecondsSquaredPerMeter),
                Tankdrive.getInstance().m_kinematics,
                Tankdrive.getInstance()::getWeelSpeeds,
                new PIDController(Constants.Testchassi.kPDriveVel, 0, 0),
                new PIDController(Constants.Testchassi.kPDriveVel, 0, 0),
                Tankdrive.getInstance()::setVolts,
                Tankdrive.getInstance());

        return command;
    }

    public RamseteCommand get_comand() {
        return getTrajectory();
    }

    public RamseteCommand start_command() {
        RamseteCommand command = getTrajectory();
        command.schedule();

        // this.setDefaultCommand(new AutoCommand(this));
        return command;
    }

    public static getAutonomousTrajectory getInstance() {
        if (instance == null) {
            instance = new getAutonomousTrajectory();
        }
        return instance;
    }
}
