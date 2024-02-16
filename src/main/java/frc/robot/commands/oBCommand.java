package frc.robot.commands;

import frc.robot.Config;
import frc.robot.subsystems.visionAutonomous.Bezier;
import frc.robot.subsystems.visionAutonomous.OwnTrajectoryGenerator;
import frc.robot.subsystems.visionAutonomous.TankdriveOdometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class oBCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final OwnTrajectoryGenerator m_subsystem;

    public oBCommand(OwnTrajectoryGenerator subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    double t;
    double dt;
    Bezier bezier;
    Pose2d startPos;
    Pose2d endPose;
    Pose2d oldPos = TankdriveOdometry.getInstance().m_odometry.getPoseMeters();
    Pose2d currentPos;

    @Override
    public void initialize() {
        t = 0;
        dt = 0.01;
        bezier = OwnTrajectoryGenerator.getInstance().generateTrajectory();
        System.out.println("start command");
        //startPos = Tankdrive_poseestimator.getInstance().m_poseEstimator.getEstimatedPosition();
        startPos = TankdriveOdometry.getInstance().m_odometry.getPoseMeters();
    }

    double[] v;

    @Override
    public void execute() {
        t += dt;

        double[] rv = bezier.get_div(t, dt);
        //double[] should_pos = bezier.get_value(t);
        currentPos = TankdriveOdometry.getInstance().m_odometry.getPoseMeters();
        
        double[] diffPos = {currentPos.getX() - oldPos.getX(), currentPos.getY()- oldPos.getY()};
        if(Math.abs(diffPos[0]) >= 1){
            diffPos[0] = 0;
        }
        if(Math.abs(diffPos[1]) >= 1){
            diffPos[1] = 0;
        }

        double[] correction = Bezier.Utils.sub(rv,diffPos);

        correction = Bezier.Utils.mult(correction, 0.01);

        //System.out.println(String.valueOf(correction[0]) + " " + String.valueOf(correction[1]));

        //rv = Bezier.Utils.add(rv,correction);

        if (Math.abs(correction[0]) + Math.abs(correction[1]) >= 0.8){
            //System.out.println(Math.abs(correction[0]) + Math.abs(correction[1]));
            //t += dt;
            
            //rv = Bezier.Utils.add(rv,correction);
            //System.out.println(String.valueOf(rv[0]) + " " + String.valueOf(rv[1]));
        }

        //System.out.println(String.valueOf(rv2[0]) + " " + String.valueOf(rv2[1]));

        rv = Bezier.Utils.mult(rv, (1/dt));

        rv = OwnTrajectoryGenerator.arcadeDrive(rv[0],rv[1]);

        Config.drive().setVolts(rv[0], rv[1]);

        TankdriveOdometry.getInstance().update_robot_pose();
        //TalonFX motor = new TalonFX(0);
        //motor.getSelectedSensorPosiion();
        
    }

    @Override
    public void end(boolean interrupted) {
        endPose = TankdriveOdometry.getInstance().m_odometry.getPoseMeters();
        System.out.println(endPose.getX()-startPos.getX());
        System.out.println(endPose.getY()-startPos.getY());
    }

    @Override
    public boolean isFinished() {
        return t >= 1;
    }
}
