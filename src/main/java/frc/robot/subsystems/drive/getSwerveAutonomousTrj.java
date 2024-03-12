package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;

import org.apache.logging.log4j.core.lookup.SystemPropertiesLookup;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.subsystems.drive.swerve_2024.SwerveDrive2024;

public class getSwerveAutonomousTrj extends SubsystemBase {

    public static getSwerveAutonomousTrj instance;

    public enum Type {
        rel,
        abs,
        futur_abs,
        futur_abs_with_waypoints
    }

    private TrajectoryConfig getTrajectoryConfig() {
        assert Config.drive().isSwerve() : "Does assert that the drive() returns a SwerveDrive";
        var kinematics = new SwerveDriveKinematics(Constants.SwerveDrive.Swerve2024.SWERVE_MODULE_TRANSLATIONS);

        SwerveDriveKinematicsConstraint constraint = new SwerveDriveKinematicsConstraint(
                kinematics, 5.2);//Constants.SwerveDrive.Swerve2024.maxVelocity.in(MetersPerSecond));// const 5,2

        TrajectoryConfig config = new TrajectoryConfig(5.2, 1.5)
                .setKinematics(kinematics)
                .addConstraint(constraint);

        return config;
    }

    private Trajectory get_future_abs_trajectory(Pose2d startPose, Pose2d endPose){
        TrajectoryConfig conf = getTrajectoryConfig();
        List<Translation2d> list_Translation2d = new ArrayList<Translation2d>();

        Trajectory new_trajectory = TrajectoryGenerator.generateTrajectory(startPose, list_Translation2d, endPose, conf);
        return new_trajectory;
    }

    private Trajectory get_future_abs_tra_waypoints(Pose2d startPose, Pose2d endPose, ArrayList<Translation2d> list){
        TrajectoryConfig conf = getTrajectoryConfig();
        List<Translation2d> list_Translation2d = list;

        Trajectory new_Trajectory = TrajectoryGenerator.generateTrajectory(startPose, list_Translation2d, endPose, conf);
        return new_Trajectory;
    }

    private Trajectory get_abs_trajectory(Pose2d endPose) {
        TrajectoryConfig conf = getTrajectoryConfig();
        List<Translation2d> list_Translation2d = new ArrayList<Translation2d>();
        Pose2d robotPose = new Pose2d(Config.drive().getPos().getTranslation(), new Rotation2d(0));

        Trajectory new_trajectory = TrajectoryGenerator.generateTrajectory(
                robotPose,
                list_Translation2d,
                endPose,
                conf);

        return new_trajectory;
    }

    private Trajectory get_rel_Trajectory(Pose2d endPose) {
        TrajectoryConfig conf = getTrajectoryConfig();
        List<Translation2d> list_Translation2d = new ArrayList<Translation2d>();

        Pose2d robotPose = new Pose2d(Config.drive().getPos().getTranslation(), new Rotation2d(0));
        Translation2d endtranslation = robotPose.getTranslation().plus(endPose.getTranslation());
        endPose = new Pose2d(endtranslation, new Rotation2d(0));
        Trajectory new_trajectory = null;
        new_trajectory = TrajectoryGenerator.generateTrajectory(Config.drive().getPos(),
                list_Translation2d,
                endPose,
                conf);

        return new_trajectory;
    }

    public Trajectory createTrajectory(Pose2d endPose, Type type) {
        Trajectory trajectory;

        if (type == Type.abs) {
            trajectory = get_abs_trajectory(endPose);
        } else if (type == Type.rel) {
            trajectory = get_rel_Trajectory(endPose);
        } else {
            throw new Error("error you have specified the wrong type of trajectory");
        }
        return trajectory;
    }

    public Trajectory createTrajectory(Pose2d startPose, Pose2d endPose, Type type) {
        Trajectory trajectory;

        if (type == Type.futur_abs) {
            trajectory = get_future_abs_trajectory(startPose, endPose);
        } else {
            throw new Error("error you have specified the wrong type of trajectory");
        }
        return trajectory;
    }

    public Trajectory createTrajectory(Pose2d startPose, Pose2d endPose, ArrayList<Translation2d> list, Type type) {
        Trajectory trajectory;

        if (type == Type.futur_abs_with_waypoints) {
            trajectory = get_future_abs_tra_waypoints(startPose, endPose, list);
        } else {
            throw new Error("error you have specified the wrong type of trajectory");
        }
        return trajectory;
    }

    public static getSwerveAutonomousTrj getInstance() {
        if (instance == null) {
            instance = new getSwerveAutonomousTrj();
        }

        return instance;
    }
}
