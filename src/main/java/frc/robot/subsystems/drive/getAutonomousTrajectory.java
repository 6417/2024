package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.autonomous_tools.PathviewerLoader;
import frc.robot.commands.CSVLoggerCommand;
import frc.robot.subsystems.vision_autonomous.Tankdrive_odometry;
import frc.robot.subsystems.vision_autonomous.Visionprocessing;

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

    public Trajectory get_abs_trajectory(Pose2d endPose){
        TrajectoryConfig conf = getTrajectoryConfig();
        List<Translation2d> list_Translation2d = new ArrayList<Translation2d>();

        Trajectory new_trajectory = TrajectoryGenerator.generateTrajectory(Tankdrive_odometry.getInstance().m_odometry.getEstimatedPosition(),
        list_Translation2d,
        endPose,
        conf);

        return new_trajectory;
    }

    public Trajectory get_rel_Trajectory(Pose2d endPose){
        TrajectoryConfig conf = getTrajectoryConfig();
        List<Translation2d> list_Translation2d = new ArrayList<Translation2d>();

        Translation2d endtranslation = Tankdrive_odometry.getInstance().m_odometry.getEstimatedPosition().getTranslation().plus(endPose.getTranslation());
        endPose = new Pose2d(endtranslation, endPose.getRotation());
        Trajectory new_trajectory = null;

        new_trajectory = TrajectoryGenerator.generateTrajectory(Tankdrive_odometry.getInstance().m_odometry.getEstimatedPosition(),
        list_Translation2d,
        endPose,
        conf);

        return new_trajectory;
    }

    public Trajectory createTrajectory(Pose2d endPose, int type) {
        Trajectory trajectory;

        if (type == 1){
            trajectory = get_abs_trajectory(endPose);
        }else if (type == 2){
            trajectory = get_rel_Trajectory(endPose);
        } else{
            trajectory = get_abs_trajectory(endPose); //shit default
        }
        return trajectory;
    }

    public static getAutonomousTrajectory getInstance() {
        if (instance == null) {
            instance = new getAutonomousTrajectory();
        }

        return instance;
    }
}
