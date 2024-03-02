package frc.robot.subsystems.drive.swerve_2019;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.joystick.Binding;
import frc.robot.abstraction.baseClasses.BDrive.DriveOrientation;
import frc.fridowpi.joystick.JoystickBindable;
import frc.robot.abstraction.baseClasses.BDrive.MountingLocations;

public class SwerveDriveBase extends SubsystemBase implements JoystickBindable {

    public DriveOrientation getDriveMode() {
        return DriveOrientation.Forwards;
    }

    public void setDriveMode(DriveOrientation driveMode) {

    }

    public void drive(ChassisSpeeds requesteSpeeds) {

    }

    public void rotateAllModules(double speed) {

    }

    public Map<MountingLocations, Boolean> areHalSensoredOfMoudlesTriggered() {
        Map<MountingLocations, Boolean> defaultReturn = new HashMap<>();
        for (var location : MountingLocations.values())
            defaultReturn.put(location, false);
        return defaultReturn;
    }

    public void stopAllMotors() {

    }

    public void setCurrentModuleRotatoinToHome(MountingLocations moduleLocation) {

    }

    public void setModuleRotationEncoderTicks(MountingLocations mountingLocation, double ticks) {

    }

    public void forEachModule(Consumer<SwerveModule> consumer) {

    }

    public SwerveModulePosition[] getOdometryPoses(){
        return new SwerveModulePosition[]{};
    }

    public boolean areAllModulesZeroed() {
        return false;
    }

    public void forEachModuleEntry(
            Consumer<Map.Entry<MountingLocations, SwerveModule>> consumer) {
    }

    public boolean isModuleZeroed(MountingLocations mountingLocation) {
        return false;
    }

    public void withModule(MountingLocations mountingLocation, Consumer<SwerveModule> consumer) {

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
