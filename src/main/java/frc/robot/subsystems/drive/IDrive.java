package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;

public interface IDrive {
    public static IDrive getInstance(){return null;};

    public void drive(double v_x, double v_y, double rot);

    public void driveToPos(Pose2d pos);
}