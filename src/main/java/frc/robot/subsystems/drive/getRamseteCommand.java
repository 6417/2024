package frc.robot.subsystems.drive;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.fridowpi.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.commands.CSVLoggerCommand;
import frc.robot.Config;

public class getRamseteCommand {

    public RamseteCommand getTrajectory(Pose2d endPose, int type) {
        Trajectory trajectory = getAutonomousTrajectory.getInstance().createTrajectory(endPose, type);
        Trajectory.State state = new Trajectory.State(0, 0, 0,
                trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters, 0);
        Trajectory waitoAtEnd = new Trajectory(List.of(state));
        trajectory = trajectory.concatenate(waitoAtEnd);

        RamseteCommand command = new RamseteCommand(
                trajectory,
                Config.drive()::getPos,
                new RamseteController(Constants.Testchassis.PathWeaver.kRamsetB,
                        Constants.Testchassis.PathWeaver.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.Testchassis.ksVolts,
                        Constants.Testchassis.kvVoltSevondsPerMeter,
                        Constants.Testchassis.kaVoltSecondsSquaredPerMeter),
                Config.drive().getDifferentialKinematics().get(),
                () -> Config.drive().getDifferentialWheelSpeeds().get(),
                new PIDController(Constants.Testchassis.kPDriveVel, 0, 0),
                new PIDController(Constants.Testchassis.kPDriveVel, 0, 0),
                Config.drive()::setVolts,
                Config.drive());

        return command;
    }

    public RamseteCommand start_command() {
        Pose2d firstApriltag = new Pose2d(15, 5.5, new Rotation2d(0));
        Pose2d backward = new Pose2d(-2, 0, new Rotation2d(0));
        Pose2d secondApriltag = new Pose2d(14.7, 7, new Rotation2d(Math.PI / 2));
        Pose2d test_back = new Pose2d(2, 0, new Rotation2d(0));

        RamseteCommand command = getTrajectory(firstApriltag, 1);
        RamseteCommand command2 = getTrajectory(backward, 2);
        RamseteCommand command3 = getTrajectory(secondApriltag, 1);
        RamseteCommand command4 = getTrajectory(test_back, 2);
        Command logComand = new ParallelRaceGroup(
                new CSVLoggerCommand("/tmp/logger.csv", getAutonomousTrajectory.getInstance().createTrajectory(firstApriltag, 1)), command);
        // command.andThen(command2).andThen(command3).schedule();

        // command.schedule();
        logComand.schedule();

        // this.setDefaultCommand(new AutoCommand(this));
        return command;
    }
}
