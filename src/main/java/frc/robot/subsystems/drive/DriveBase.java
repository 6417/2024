package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase implements IDrive {

    private static DriveBase instance;

    protected DriveBase(){
        //intialiser
    }
    
    @Override
    public void periodic() {}

    public void drive(double v_x, double v_y, double rot){}

    public void driveToPos(Pose2d pos){}
}
