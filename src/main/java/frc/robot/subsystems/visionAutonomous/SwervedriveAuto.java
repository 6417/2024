package frc.robot.subsystems.visionAutonomous;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import frc.fridowpi.module.Module;
import frc.robot.Config;
import frc.robot.commands.SwervedriveAutoCommand;
import frc.robot.subsystems.drive.getSwerveAutonomousTrj;
import frc.robot.subsystems.drive.getSwerveAutonomousTrj.Type;

///////////// TODO /////////////
// set start velocity
// autonomous
// behave with fieldoriented

//////////// NOTES /////////////
// manipulating trajectorys


public class SwervedriveAuto extends Module {

    Pose2d source;
    Pose2d amp;
    Pose2d speaker;

	ChassisSpeeds speeds = new ChassisSpeeds();
    static SwervedriveAuto instance;

    //guest values with alsow the constraints in the constants for the trajectory generation
    PIDController pid = new PIDController(11,0.12,0.02);
    private HolonomicDriveController controller = new HolonomicDriveController(
        pid,pid,
        new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)));

    private SwervedriveAuto(){
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            source = new Pose2d(null, null);
            amp = new Pose2d(null,null);
            speaker = new Pose2d(null,null);
        }
        else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
            source = new Pose2d(null, null);
            amp = new Pose2d(null, null);
            speaker = new Pose2d(null, null);
        }
    }

    // this method will be called from the command to get the speeds
    public ChassisSpeeds getVelocities(Trajectory tra, double t){
        Trajectory.State goal = tra.sample(t);
        Rotation2d rot = goal.poseMeters.getRotation();
        speeds = controller.calculate(Config.drive().getPos(), goal, rot);
        return speeds;
    }

    public void driveToSource(){
        Trajectory tra = getSwerveAutonomousTrj.getInstance().createTrajectory(source, Type.abs);
        startCommand(tra);
    }

    public void driveToAmp(){
        Trajectory tra = getSwerveAutonomousTrj.getInstance().createTrajectory(amp, Type.abs);
        startCommand(tra);
    }

    public void driveToSpeaker(){
        Trajectory tra = getSwerveAutonomousTrj.getInstance().createTrajectory(speaker, Type.abs);
        startCommand(tra);
    }

    private void startCommand(Trajectory tra){
        SwervedriveAutoCommand command = new SwervedriveAutoCommand(tra);
        command.schedule();
    }

    // attention not correcect whenn making sequentialn and still standing at the same position
    public SwervedriveAutoCommand getCommand(Trajectory tra){
        return new SwervedriveAutoCommand(tra);
    }

    public void startCommand(){

        Pose2d firstApriltag = new Pose2d(15, 5.5, new Rotation2d(0));
<<<<<<< HEAD
        Pose2d test = new Pose2d(1, 0,new Rotation2d(0));
=======
        Pose2d test = new Pose2d(3, 0,new Rotation2d(0));

        //test trajectorys
        Trajectory tra2 = getSwerveAutonomousTrj.getInstance().createTrajectory(firstApriltag, Type.abs);
        Trajectory testtra = getSwerveAutonomousTrj.getInstance().createTrajectory(test, Type.rel);
>>>>>>> 222ca1fea748151204a049f3cbb2ffb1f292f056

        SwervedriveAutoCommand command = new SwervedriveAutoCommand(testtra);
        command.schedule();
    }

    public static SwervedriveAuto getInstance(){
        if (instance == null){
            instance = new SwervedriveAuto();
        }
        return instance;
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
