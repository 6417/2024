package frc.robot.subsystems.visionAutonomous;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.subsystems.drive.getSwerveAutonomousTrj;
import frc.robot.subsystems.drive.getSwerveAutonomousTrj.Type;

public class Autonomous extends SubsystemBase {

    // achtung man kann warscheinlich die rotationen nehmen weil sie nicht absolut, sondern relativ sind

    //blue autonomous
    Pose2d bluespeaker = new Pose2d(0.45, 2.64, new Rotation2d(180));
    Pose2d blueend1 = new Pose2d(4.419, 1.945, new Rotation2d(0));
    Pose2d blueend2 = new Pose2d(4.419,5.4, new Rotation2d(0));
    Pose2d blueamp = new Pose2d(1.8,7.77, new Rotation2d(90));
    Pose2d bluestart1 = new Pose2d(0.92,3.1, new Rotation2d(0));
    Pose2d bluestart2 = new Pose2d(0.92,2.1, new Rotation2d(0));
    ArrayList<Translation2d> redpoints1 = new ArrayList<>(); //for ampToEnd1

    //red autonomous
    Pose2d redspeakter = new Pose2d(15.18,5.4, new Rotation2d(180));
    Pose2d redend1 = new Pose2d(12.34,6.3, new Rotation2d(0));
    Pose2d redend2 = new Pose2d(12.34, 1.2, new Rotation2d(0));
    Pose2d redamp = new Pose2d(14.74,7.77, new Rotation2d(90));
    Pose2d redstart1 = new Pose2d(15.62,3.1, new Rotation2d(0));
    Pose2d redstart2 = new Pose2d(15.62,2.1 , new Rotation2d(0));
    ArrayList<Translation2d> bluePoints1 = new ArrayList<>(); // for ampToEnd1

    ArrayList<Translation2d> emptylsit = new ArrayList<>();


    public Command followPath(Pose2d start, Pose2d end, ArrayList<Translation2d> list, Rotation2d endRot) {
        Trajectory first_trajectory = getSwerveAutonomousTrj.getInstance()
                .createTrajectory(start, end, list,Type.futur_abs_with_waypoints);
        return Config.active.getAuto().get().getAutoCommand(first_trajectory, endRot);
    }
    
    public Command blueSpeakerToEnd1() {
        return new SequentialCommandGroup(
            //new ShootSpekerCommand(),
            followPath(redspeakter, redend1, emptylsit, redend1.getRotation()));}

    public Command blueSpeakerToEnd2(){
        return new SequentialCommandGroup(
            //new ShooterSpeakerCommand(),
            followPath(redspeakter, redend2, emptylsit, redend2.getRotation()));}

    public Command blueAmpToEnd1(){
        redpoints1.add(new Translation2d(2.1,6.3));
        return new SequentialCommandGroup(
            //new ShooterAmpCommand(),
            followPath(redamp, redend1, emptylsit, redend1.getRotation()));}

    public Command blueStart1ToSpeakerToEnd1(){
        return new SequentialCommandGroup(
            followPath(redstart1, redspeakter, emptylsit, redspeakter.getRotation()),
            //new ShooterSpeakerCommand(),
            followPath(redspeakter, redend1, emptylsit, redend1.getRotation()));}

    public Command blueStart1ToAmpToEnd1(){
        return new SequentialCommandGroup(
            followPath(redstart1, redamp, emptylsit, redamp.getRotation()),
            //new ShooterSpekerCommand(),
            followPath(redamp, redend1, emptylsit, redend1.getRotation()));}
    
    public Command blueStart2ToAmpToEnd1(){
        return new SequentialCommandGroup(
            followPath(redstart2, redspeakter, emptylsit, redspeakter.getRotation()),
            //new ShooterSpeakerCommand(),
            followPath(redspeakter, redend1, emptylsit, redend1.getRotation()));}

    public Command redSpeakerToEnd1(){
        return new SequentialCommandGroup(
            //new ShooterSpeakerCommand(),
            followPath(bluespeaker, blueend1, emptylsit, blueend1.getRotation()));}

    public Command redSpeakerToEnd2(){
        return new SequentialCommandGroup(
            //new ShooterSpeakerCommand(),
            followPath(bluespeaker, blueend2, emptylsit, blueend2.getRotation()));}
        
    public Command redAmpToEnd1(){
        bluePoints1.add(new Translation2d(14.44,6.3));
        return new SequentialCommandGroup(
            //new ShooterAmpCommand(),
            followPath(blueamp, blueend1, emptylsit, blueend1.getRotation()));}

    public Command redStart1ToSpeakerToEnd1(){
        return new SequentialCommandGroup(
            followPath(bluestart1, bluespeaker, emptylsit, bluespeaker.getRotation()),
            // new ShooterSpeakerCommand(),
            followPath(bluespeaker, blueend1, emptylsit, blueend1.getRotation()));}

    public Command redStart2ToSpeakerToEnd1(){
        return new SequentialCommandGroup(
            followPath(bluestart2, bluespeaker, emptylsit, bluespeaker.getRotation()),
            // new ShooterSpeakerCommand(),
            followPath(bluespeaker, blueend1, emptylsit, blueend1.getRotation()));}

    public Autonomous() {
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
