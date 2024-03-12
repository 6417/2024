package frc.robot.subsystems.visionAutonomous;

import java.util.ArrayList;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.fridowpi.module.Module;
import frc.robot.Config;
import frc.robot.abstraction.baseClasses.BAutoHandler;
import frc.robot.commands.SwervedriveAutoCommand;
import frc.robot.subsystems.drive.getSwerveAutonomousTrj;
import frc.robot.subsystems.drive.getSwerveAutonomousTrj.Type;

///////////// TODO /////////////
// set start velocity

//////////// NOTES /////////////
// manipulating trajectorys

public class SwervedriveAuto extends BAutoHandler {

    Pose2d source;
    Pose2d amp;
    Pose2d speaker;
    Rotation2d endSourceRotation;
    Rotation2d endAmpRotation;
    Rotation2d endSpeakerRotation;

    ChassisSpeeds speeds = new ChassisSpeeds();
    static SwervedriveAuto instance;

    // guest values with alsow the constraints in the constants for the trajectory
    // generation
    PIDController pid = new PIDController(0.1, 0, 0);//0.8, 0.01
    private HolonomicDriveController controller = new HolonomicDriveController(
            pid, pid,
            new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)));

    public SwervedriveAuto() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            source = new Pose2d(null, null);
            amp = new Pose2d(null, null);
            speaker = new Pose2d(null, null);
            endSourceRotation = new Rotation2d(0);
            endSpeakerRotation = new Rotation2d(0);
            endAmpRotation = new Rotation2d(0);

        } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            source = new Pose2d(null, null);
            amp = new Pose2d(null, null);
            speaker = new Pose2d(null, null);
            endSourceRotation = new Rotation2d(0);
            endSpeakerRotation = new Rotation2d(0);
            endAmpRotation = new Rotation2d(0);
        }
    }

    // this method will be called from the command to get the speeds
    @Override
    public ChassisSpeeds getVelocitiesAtTimepoint(Trajectory tra, double t, Rotation2d endRot) {
        Trajectory.State goal = tra.sample(t);
        Pose2d rp = Config.drive().getPos();
        Pose2d relativPose2d = new Pose2d(rp.getX()+ (goal.poseMeters.getY() - rp.getY()), 
        rp.getY() + (goal.poseMeters.getX() - rp.getX()), new Rotation2d(0));
        speeds = controller.calculate(rp, relativPose2d, goal.velocityMetersPerSecond,  endRot);
        return speeds;
    }

    // alsow called by the command
    public ChassisSpeeds getVelocitiesToPose(Pose2d pose, Rotation2d endRot) {
        Pose2d rp = Config.drive().getPos();
        Pose2d relativRobotPose2d = new Pose2d(rp.getX() + (pose.getY() - rp.getY()),
                rp.getY() + (pose.getX() - rp.getX()), new Rotation2d(0));
        speeds = controller.calculate(rp, relativRobotPose2d, 0.0, endRot);
        return speeds;
    }

    // drive commands to drive to destination
    public void driveToSource() {
        Trajectory tra = getSwerveAutonomousTrj.getInstance().createTrajectory(source, Type.abs);
        getAutoCommand(tra, endSourceRotation).schedule();
    }

    public void driveToAmp() {
        Trajectory tra = getSwerveAutonomousTrj.getInstance().createTrajectory(amp, Type.abs);
        getAutoCommand(tra, endAmpRotation).schedule();
    }

    public void driveToSpeaker() {
        Trajectory tra = getSwerveAutonomousTrj.getInstance().createTrajectory(speaker, Type.abs);
        getAutoCommand(tra, endSpeakerRotation).schedule();
    }

    @Override
    public Command getAutoCommand(Trajectory tra, Rotation2d endRot) {
        return new SwervedriveAutoCommand(tra, endRot);
    }

    public Command getPoseCommand(Pose2d pose, Rotation2d endRot) {
        return new SwervedriveAutoCommand(pose, endRot);
    }

    public Command getAutoCommand() {

        Pose2d firstApriltag = new Pose2d(15, 5.5, new Rotation2d(0));
        Pose2d test = new Pose2d(2, 0, new Rotation2d(0));
        ArrayList<Translation2d> points = new ArrayList<>();
        // points.add(new Translation2d(1,-1));

        // test trajectorys
        Trajectory tra2 = getSwerveAutonomousTrj.getInstance().createTrajectory(firstApriltag, Type.abs);
        Trajectory testtra = getSwerveAutonomousTrj.getInstance().createTrajectory(Config.drive().getPos(),
                test, points, Type.futur_abs_with_waypoints);

        SwervedriveAutoCommand command = new SwervedriveAutoCommand(testtra, new Rotation2d(0));
        // return command;

        return Autonomous.getInstance().blueSpeakerToEnd1();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("SpeedsX", () -> speeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("SpeedsY", () -> speeds.vyMetersPerSecond, null);
        builder.addDoubleProperty("SpeedsRot", () -> speeds.omegaRadiansPerSecond, null);

        builder.addDoubleProperty("ActualX", () -> Config.drive().getPos().getX(), null);
        builder.addDoubleProperty("ActualY", () -> Config.drive().getPos().getY(), null);
        builder.addDoubleProperty("actualRot", () -> Config.drive().getPos().getRotation().getDegrees(), null);
    }
}
