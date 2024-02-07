package frc.robot.subsystems.drive.tankdrive;

import frc.fridowpi.motors.FridoFalcon500;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.robot.abstraction.baseClasses.BDrive;

/* Base class for tankdrives with four motors */
public abstract class FourMotorTankdrive extends BDrive {
    public enum MotorRole {
        LeftMaster, RightMaster,
        LeftFollower, RightFollower
    }

    public class MotorSet {
        FridolinsMotor leftMaster;
        FridolinsMotor rightMaster;
        FridolinsMotor leftFollower;
        FridolinsMotor rightFollower;

        public MotorSet(
                int leftMasterId, int rightMasterId,
                int leftFollowerId, int rightFollowerId) {
            leftMaster = new FridoFalcon500(leftMasterId);
            rightMaster = new FridoFalcon500(rightMasterId);
            leftFollower = new FridoFalcon500(leftFollowerId);
            rightFollower = new FridoFalcon500(rightFollowerId);
            // Default to following without inversion
            rightFollower.follow(rightMaster, DirectionType.followMaster);
            leftFollower.follow(leftMaster, DirectionType.followMaster);
        }

        /* Invert motor with a given role, while keeping the others the same */
        public void invert(MotorRole role) {
            var m = getMotorByRole(role);
            switch (role) {
                // If master, invert it but revert direction of the follower
                case LeftMaster:
                    m.setInverted(true);
                    leftFollower.follow(m, DirectionType.invertMaster);
                case RightMaster:
                    m.setInverted(true);
                    rightFollower.follow(m, DirectionType.invertMaster);
                    break;
                case LeftFollower:
                    leftFollower.follow(m, DirectionType.invertMaster);
                    break;
                case RightFollower:
                    rightFollower.follow(m, DirectionType.invertMaster);
                    break;
            }
            getMotorByRole(role).setInverted(true);
        }

        FridolinsMotor getMotorByRole(MotorRole motor) {
            switch (motor) {
                case LeftMaster:
                    return leftMaster;
                case RightMaster:
                    return rightMaster;
                case LeftFollower:
                    return leftFollower;
                case RightFollower:
                    return leftFollower;
            }
            // Not possible, as long as all enum variants are handled above
            return null;
        }
    }

    MotorSet motors;

    public FourMotorTankdrive(
            int leftMaster, int rightMaster,
            int leftFollower, int rightFollower) {
        motors = new MotorSet(
                leftMaster, rightMaster,
                leftFollower, rightFollower);
    }
}
