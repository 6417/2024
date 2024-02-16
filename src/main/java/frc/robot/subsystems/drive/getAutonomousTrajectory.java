package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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

    private RamseteCommand getTrajectory(Pose2d endPose, int type) {

        Trajectory trajectory = createTrajectory(endPose, type);

        Trajectory.State state = new Trajectory.State(5, 0, 0, trajectory.getStates().get(trajectory.getStates().size()-1).poseMeters, 0);
        Trajectory waitoAtEnd = new Trajectory(List.of(state));
        trajectory = trajectory.concatenate(waitoAtEnd);
        //Trajectory drive_to_apriltag = get_abs_trajectory(new Pose2d(15.5,5.5, new Rotation2d(0)));¨
        
        //Trajectory own_trajectory_from_pathviewer = PathviewerLoader.loadTrajectory("C:/Users/l.hefti/Desktop/2024/PathWeaver/pathweaver.json");

        RamseteCommand command = new RamseteCommand(
                trajectory,
                Tankdrive.getInstance()::getPos,
                new RamseteController(Constants.Testchassi.PathWeaver.kRamsetB,Constants.Testchassi.PathWeaver.kRamseteZeta),
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

    private Trajectory createTrajectory(Pose2d endPose, int type) {
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

    public RamseteCommand start_command() {
        Pose2d firstApriltag = new Pose2d(15,5.5,new Rotation2d(0));
        Pose2d backward = new Pose2d(-2,0, new Rotation2d(0));
        Pose2d secondApriltag = new Pose2d(14.7,7,new Rotation2d(Math.PI/2));
        Pose2d test_back = new Pose2d(2, 0, new Rotation2d(0));

        RamseteCommand command = getTrajectory(firstApriltag,1);
        RamseteCommand command2 = getTrajectory(backward, 2);
        RamseteCommand command3 = getTrajectory(secondApriltag,1);
        RamseteCommand command4 = getTrajectory(test_back, 2);
        Command logComand = new ParallelRaceGroup(new CSVLoggerCommand("/tmp/logger.csv", createTrajectory(firstApriltag, 1)),command);
        //command.andThen(command2).andThen(command3).schedule();
        
        //command.schedule();
        logComand.schedule();


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
