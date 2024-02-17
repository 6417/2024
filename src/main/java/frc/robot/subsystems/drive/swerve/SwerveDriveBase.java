package frc.robot.subsystems.drive.swerve;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.JoystickBindable;
import frc.robot.Constants;

public class SwerveDriveBase extends SubsystemBase implements JoystickBindable {
    public enum DriveOrientation {
        FieldOriented, ShooterFront, ShooterBack
    }

    public DriveOrientation getDriveMode() {
        return DriveOrientation.ShooterBack;
    }

    public void setDriveMode(DriveOrientation driveMode) {

    }

    public void drive(ChassisSpeeds requesteSpeeds) {

    }

    public void rotateAllModules(double speed) {

    }

    public Map<Constants.SwerveDrive.MountingLocations, Boolean> areHalSensoredOfMoudlesTriggered() {
        Map<Constants.SwerveDrive.MountingLocations, Boolean> defaultReturn = new HashMap<>();
        for (var location : Constants.SwerveDrive.MountingLocations.values())
            defaultReturn.put(location, false);
        return defaultReturn;
    }

    public void stopAllMotors() {

    }

    public void setCurrentModuleRotatoinToHome(Constants.SwerveDrive.MountingLocations moduleLocation) {

    }

    public void setModuleRotationEncoderTicks(Constants.SwerveDrive.MountingLocations mountingLocation, double ticks) {

    }

    public void forEachModule(Consumer<SwerveModule> consumer) {

    }

    public boolean areAllModulesZeroed() {
        return false;
    }

    public void forEachModuleEntry(
            Consumer<Map.Entry<Constants.SwerveDrive.MountingLocations, SwerveModule>> consumer) {
    }

    public boolean isModuleZeroed(Constants.SwerveDrive.MountingLocations mountingLocation) {
        return false;
    }

    public void withModule(Constants.SwerveDrive.MountingLocations mountingLocation, Consumer<SwerveModule> consumer) {

    }

    public void setSpeedFactor(double speedFactor) {

    }

    public void configureButtonBindings(Joystick joystick) {

    }

    public SwerveDriveKinematics getKinematics(){
        return null;
    }

    @Override
    public List<Binding> getMappings() {
		return List.of();
    }
}
