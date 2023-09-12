package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase{
    private static PhotonVision pvInstance;

    private NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private NetworkTable table = instance.getTable("limelightCam");
    
    PhotonCamera camera = new PhotonCamera("limelightCam");
    PhotonPipelineResult result = getLatestPipeline();

    static final double CAMERA_HEIGHT_METERS = 0.16;
    static final double TARGET_HEIGHT_METERS = 0.4699;
    static final double CAMERA_YAW = 28;
    //Multiply to pitch in getDistance();
    static final double AAGRIMS_CONSTANT = 1.6214288372574428;
    //Add to pitch in getDistance();
    static final double YAJWINS_CONSTANT = -3.0414949031355434;
    //Multiply to yaw in getY();
    static final double ARYAVS_CONSTANT = -0.04146708961342199;
    //Add to yaw in getY();
    static final double RITVIKS_CONSTANT = -0.5709238051120222;

    public static PhotonVision getInstance() {
        if (pvInstance == null) {
            pvInstance = new PhotonVision();
        }

        return pvInstance;
    }

    /**
     * have to use same pipeline result each time you want to gather data
     * gets the processed data from the camera
     * @return the latest pipeline result
     */
    public PhotonPipelineResult getLatestPipeline(){
        return camera.getLatestResult();
    }
    
    //Checks if there is a target in vision
    public boolean hasTarget(PhotonPipelineResult result){
        return result.hasTargets();
    }

    /**
     * gets the targets
     * @param result stores data
     * @returna copy of the vector of targets
     */
    public List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result){
        return result.getTargets();
    }

    public PhotonTrackedTarget getBestTarget(PhotonPipelineResult result){
        return result.getBestTarget();
    }

    //The yaw of the target in degrees (positive right)
    public double getYaw(PhotonTrackedTarget target){
        return target.getYaw();
    }

    //The pitch of the target in degrees (positive up)
    public double getPitch(PhotonTrackedTarget target){
        return target.getPitch();
    }

    //The area (how much of the camera feed the bounding box takes up) as a percent (0-100)
    public double getArea(PhotonTrackedTarget target){
        return target.getArea();
    }

    //The skew of the target in degrees (counter-clockwise positive)
    public double getSkew(PhotonTrackedTarget target){
        return target.getSkew();
    }

    //The 4 corners of the minimum bounding box rectangle
    public List<TargetCorner> getBoundingCorners(PhotonTrackedTarget target){
        return target.getDetectedCorners();
    }

    //The camera to target transform (Pose)
    //For some reason cannot get pose for reflectiveTape
    // public Transform2d getPose(PhotonTrackedTarget target){
    //     return target.getCameraToTarget();
    // }


    //Get id of tag
    public int getTargetId(PhotonTrackedTarget target){
        return target.getFiducialId();
    }

    //How ambiguous the pose is????
    public double getPoseAbmiguity(PhotonTrackedTarget target){
        return target.getPoseAmbiguity();
    }

    /*Get the transform that maps camera space (X = forward, Y = left, Z = up) 
    to object/fiducial tag space (X forward, Y left, Z up) with the lowest reprojection error*/
    public Transform3d getBestCamera(PhotonTrackedTarget target){
        return target.getBestCameraToTarget();
    }

    /*Get the transform that maps camera space (X = forward, Y = left, Z = up) 
    to object/fiducial tag space (X forward, Y left, Z up) with the lowest highest error*/
    public Transform3d getAlternateCamera(PhotonTrackedTarget target){
        return target.getAlternateCameraToTarget();
    }

    public double getDistance(){
        double dist = 0.0;
       
        if(result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();
            dist = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, 
            TARGET_HEIGHT_METERS, Units.degreesToRadians(CAMERA_YAW), Units.degreesToRadians((AAGRIMS_CONSTANT * target.getPitch() + YAJWINS_CONSTANT)));   
        }
        else{
            dist = 0.0;
        }
        return dist;

    }

    public double getY(){
        double yaw = 0;
        if(result.hasTargets()){
            yaw = getYaw(result.getBestTarget());
        }
        return -getDistance() * Math.tan(Units.degreesToRadians(AAGRIMS_CONSTANT * yaw + YAJWINS_CONSTANT));
    }

    public double getX(){
        return getDistance();
    }

    @Override
    public void periodic(){
        result = getLatestPipeline();
        if (result.hasTargets()) {
            SmartDashboard.putNumber("Actual Distance", getDistance());
            SmartDashboard.putNumber("vertical distance to target", getX());
            SmartDashboard.putNumber("horizontal distance to target", getY());
            SmartDashboard.putNumber("target pitch: ", result.getBestTarget().getPitch());
            SmartDashboard.putNumber("target yaw: ", result.getBestTarget().getYaw());
        }

    }

}
