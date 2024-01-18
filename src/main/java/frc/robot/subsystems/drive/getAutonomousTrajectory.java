package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Tankdrive_poseestimator;

public class getAutonomousTrajectory {

    public static getAutonomousTrajectory instance;

    private TrajectoryConfig getTrajectoryConfig(){
    var voltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.Testchassi.ksVolts,
            Constants.Testchassi.kvVoltSevondsPerMeter, 
            Constants.Testchassi.kaVoltSecondsSquaredPerMeter),Tankdrive.getInstance().m_kinematics, 10);

    TrajectoryConfig config = new TrajectoryConfig(Constants.Testchassi.PathWeaver.kMaxVMetersPerSecond,
    Constants.Testchassi.PathWeaver.kMaxAccMetersPerSecond)
    .setKinematics(Tankdrive.getInstance().m_kinematics)
    .addConstraint(voltageConstraint);

    return config;
    }

    private RamseteCommand getTrajectory(){
        TrajectoryConfig conf = getTrajectoryConfig();
        List<Translation2d> list_translationd2 = new ArrayList<Translation2d>();

        Trajectory new_trajectory = TrajectoryGenerator.generateTrajectory(Tankdrive_poseestimator.getInstance().m_poseEstimator.getEstimatedPosition(),
        list_translationd2, Tankdrive_poseestimator.getInstance().m_poseEstimator.getEstimatedPosition().plus(new Transform2d(new Translation2d(1,0),new Rotation2d(0))),conf);

        RamseteCommand command = new RamseteCommand(
            new_trajectory,
            Tankdrive.getInstance()::getPos,
            new RamseteController(Constants.Testchassi.PathWeaver.kRamsetB, Constants.Testchassi.PathWeaver.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.Testchassi.ksVolts,
                Constants.Testchassi.kvVoltSevondsPerMeter,
                Constants.Testchassi.kaVoltSecondsSquaredPerMeter),
            Tankdrive.getInstance().m_kinematics,
            Tankdrive.getInstance()::getWeelSpeeds,
            new PIDController(Constants.Testchassi.kPDriveVel,0,0),
            new PIDController(Constants.Testchassi.kPDriveVel, 0,0),
            Tankdrive.getInstance()::setVolts,
            Tankdrive.getInstance()
        );

        return command;
    }

    public void execute(){
        RamseteCommand command = getTrajectory();
        command.schedule();
    }

    public static getAutonomousTrajectory getInstance(){
        if (instance == null){
            instance = null;
        }
        return instance;
    }
}
