
package frc.robot.subsystems.vision_autonomous;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Tankdrive;

public class Own_trajectory_generytor extends SubsystemBase {

    public static Own_trajectory_generytor instance;

    public Own_trajectory_generytor() {
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

    public static Own_trajectory_generytor getInstance() {
        if (instance == null) {
            instance = new Own_trajectory_generytor();
        }
        return instance;
    }
}
