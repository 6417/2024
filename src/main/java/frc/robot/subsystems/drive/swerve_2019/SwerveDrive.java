package frc.robot.subsystems.drive.swerve_2019;

public class SwerveDrive {
    static SwerveDrive instance = new SwerveDrive();
    private SwerveDrive() { }
    public static SwerveDrive getInstance() {
        return instance;
    }
}
