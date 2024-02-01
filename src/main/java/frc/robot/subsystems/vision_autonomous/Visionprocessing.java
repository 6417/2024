package frc.robot.subsystems.vision_autonomous;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Visionprocessing extends SubsystemBase {

    static Visionprocessing instance;

    double[] apriltag_position = { 15.079472, 0.245872, 1.355852,
            16.185134, 0.883666, 1.355852,
            16.579342, 4.982718, 1.451102,
            16.579342, 5.547868, 1.451102,
            14.700758, 8.2042, 1.355852,
            1.8415, 8.2042, 1.355852,
            -0.0381, 5.547868, 1.451102,
            -0.0381, 4.982718, 1.451102,
            0.356108, 0.883666, 1.355852,
            1.461516, 0.245872, 1.355852,
            11.904726, 3.713226, 1.3208,
            11.904726, 4.49834, 1.3208,
            11.220196, 4.105148, 1.3208,
            5.320792, 4.105148, 1.3208,
            4.641342, 4.49834, 1.3208,
            4.641342, 3.713226, 1.3208 };

    double[] apriltag_rotation = { 120, 120, 180, 180, 270, 270, 0, 0, 60, 60, 300, 60,
            180, 0, 120, 240 };

    private Visionprocessing() {
    }

    @Override
    public void periodic() {
    }

    private double[] getData() {
        double[] data = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        return data;
    }

    private int getId(){
        double id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(-1);
        return (int) id;
    }

    public int validTarget(){
        double t = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getInteger(-1);
        return (int) t;
    }

    public double[] getFieldPos(){
        double[] data = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]); //default botpose
        return data;
    }

    private double[] get_relativ_robotPose(double[] pose){
        return pose;
    }

    private double[] converte_relativ_pose_to_absolute(double[] pose, int id){
        double rotation = apriltag_rotation[id-1]+90;
        double[] abs_pose = {pose[0]*Math.sin(rotation) + pose[0]*Math.cos(rotation), 
            pose[1] * (-Math.sin(rotation)) + pose[1]*(-Math.cos(rotation)),
            pose[2],
            pose[3],
            pose[4],
            pose[5]};
        return abs_pose;
    }
    
    private double[] sub_vec(double[] tag_pose, double[] abs_vec_pos){
        double[] subvec = {tag_pose[0]-abs_vec_pos[0],tag_pose[1]-abs_vec_pos[1],abs_vec_pos[2],abs_vec_pos[3],abs_vec_pos[4],abs_vec_pos[5]};
        return subvec;
    }

    public double[] get_abs_pose_on_field(){
        double[] apriltag_pos = getFieldPos();
        int id = getId();
        
        double[] abs_pose_vectors = converte_relativ_pose_to_absolute(apriltag_pos, id);
        double[] tag_pose = {apriltag_position[(id-1)*3],apriltag_position[(id-1)*3 + 1],apriltag_position[(id-1)*3 + 2]};
        double[] field_pos = sub_vec(tag_pose,abs_pose_vectors);
        return field_pos;
    }

    public static Visionprocessing getInstance() {
        if (instance != null) {
            return instance;
        } else {
            instance = new Visionprocessing();
            return instance;
        }
    }
}
