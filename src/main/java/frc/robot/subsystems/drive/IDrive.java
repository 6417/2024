package frc.robot.subsystems.drive;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface IDrive {
    // Steering functions
    public void drive(double v_x, double v_y, double rot);

    public void setVolts(double leftvolts, double rigthvolts);

    public void driveToPos(Pose2d pos);

    // Braking
    public void release_brake();

    public void brake();

    // Sysid tuning (not really necessary)
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction);

    public Command sysIdDynamic(SysIdRoutine.Direction direction);

    // Getters
    public Pose2d getPos();

    public double getLeftEncoderPos();

    public double getRightEncoderPos();

    // Couldn't (yet) generalize those
    public Optional<DifferentialDriveKinematics> getDifferentialKinematics();
    public Optional<SwerveDriveKinematics> getSwerveKinematics();

    // Abstraction stuff
    public boolean isSwerve();

    // Must be sendable
    public void initSendable(SendableBuilder builder);
}