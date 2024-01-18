package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DriveBase extends SubsystemBase implements IDrive {

    private static DriveBase instance;

    protected DriveBase(){
        //intialiser
    }
    
    @Override
    public void periodic() {}

    public void drive(double v_x, double v_y, double rot){}

    public void driveToPos(Pose2d pos){}
    
    public Pose2d getPos() {
        return null;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
        return null;
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction){
        return null;
    }
}
