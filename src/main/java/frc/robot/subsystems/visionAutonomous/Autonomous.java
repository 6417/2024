package frc.robot.subsystems.visionAutonomous;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.subsystems.drive.getSwerveAutonomousTrj;
import frc.robot.subsystems.drive.getSwerveAutonomousTrj.Type;

public class Autonomous extends SubsystemBase {

  private class Path {
    public ArrayList<Pose2d> poses = new ArrayList<>();
    public ArrayList<Trajectory> tra = new ArrayList<>();

    public void generatePath() {
      if (poses.size() <= 1) {
        throw new Error("error in generate Path");
      }
      for (int i = 1; i < poses.size(); i++) {
        tra.add(getSwerveAutonomousTrj.getInstance().createTrajectory(poses.get(i - 1), poses.get(i), Type.futur_abs));
      }
    }
  }

  Path examplepath = new Path();

  public Autonomous() {
    examplepath.poses.add(Config.drive().getPos());
    examplepath.poses.add(new Pose2d(1, 1, new Rotation2d(0)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
