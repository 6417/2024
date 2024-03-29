package frc.robot.subsystems.drive.swerve_2019;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveKinematics<MountingLocaiton extends Enum<MountingLocaiton>> extends SwerveDriveKinematics {
    private Map<Integer, MountingLocaiton> indicies;

    @SuppressWarnings("unchecked")
    public SwerveKinematics(Map<MountingLocaiton, Translation2d> locations) {
        super(locations.values().toArray(Translation2d[]::new));
        indicies = new HashMap<>();
        for (int i = 0; i < locations.size(); i++) {
            MountingLocaiton key = ((Entry<MountingLocaiton, Translation2d>) locations.entrySet().toArray()[i])
                    .getKey();
            indicies.put(i, key);
        }
    }

    public Map<MountingLocaiton, SwerveModuleState> toLabledSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = super.toSwerveModuleStates(chassisSpeeds);
        Map<MountingLocaiton, SwerveModuleState> labeledStates = new HashMap<>();
        for (int i = 0; i < states.length; i++)
            labeledStates.put(indicies.get(i), states[i]);

        return labeledStates;
    }
}
