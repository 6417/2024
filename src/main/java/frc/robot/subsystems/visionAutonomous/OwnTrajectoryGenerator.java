package frc.robot.subsystems.visionAutonomous;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OwnTrajectoryGenerator extends SubsystemBase {

    public static OwnTrajectoryGenerator instance;

    public OwnTrajectoryGenerator() {
    }

    public Bezier generateTrajectory(){
        double[] startP = {0,0};
        List<double[]> pos = new ArrayList<>();
        double[] p1 = {1,1};
        pos.add(p1);
        double[] endP = {2,0};
        Bezier trajectory = new Bezier(startP,pos,endP);
        return trajectory;
    }

    @Override
    public void periodic() {
    }

    public static double[] arcadeDrive(double v_x, double rot){
        double[] res = {v_x + rot, v_x - rot};
        return res;
    }

    public static OwnTrajectoryGenerator getInstance() {
        if (instance == null) {
            instance = new OwnTrajectoryGenerator();
        }
        return instance;
    }
}
