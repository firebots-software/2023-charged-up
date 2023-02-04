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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase{
    private NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private NetworkTable table = instance.getTable("limelightCam");
    
    PhotonCamera camera = new PhotonCamera("limelightCam");

    static double CAMERA_HEIGHT_METERS = 0.6477;
    static double TARGET_HEIGHT_METERS = 1.17602;
    static double CAMERA_YAW = 24.5;

    private PhotonPipelineResult pipeline;

    //Have to use the same pipeline result each time you want to gather data.
    //Gets the processed data from the camera
    public PhotonPipelineResult getLatestPipeline(){
        return camera.getLatestResult();
    }
    //Checks if there is a target in vision
    public boolean hasTarget(PhotonPipelineResult result){
        return result.hasTargets();
    }

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
        if (target.getBestCameraToTarget() == null) {
            System.out.println("null transform");
        }
        return target.getBestCameraToTarget();
    }

    /*Get the transform that maps camera space (X = forward, Y = left, Z = up) 
    to object/fiducial tag space (X forward, Y left, Z up) with the lowest highest error*/
    public Transform3d getAlternateCamera(PhotonTrackedTarget target){
        if (target.getBestCameraToTarget() == null) {
            System.out.println("null transform3d");
        }
        return target.getAlternateCameraToTarget();
    }

    public double getDistance(){
        PhotonTrackedTarget target = pipeline.getBestTarget();
        double dist = 0.0;
       
        if(pipeline.hasTargets()){
            dist = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, 
            TARGET_HEIGHT_METERS, Units.degreesToRadians(CAMERA_YAW), Units.degreesToRadians(target.getPitch()));   
        }
        else{
            dist = 0.6;
        }
        return dist;

    }

    
    public double getY(){
        double dist = 0;
        double yaw = 0;
        double pitch = 0;

        if(pipeline.hasTargets()){
            yaw = getYaw(pipeline.getBestTarget());
            pitch = getPitch(pipeline.getBestTarget());
            dist = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, 
            TARGET_HEIGHT_METERS, Units.degreesToRadians(CAMERA_YAW), Units.degreesToRadians(pitch));   
        }

        return Math.abs((dist) / Math.cos(yaw) - 0.37);
    }
    public double getX(){
        double dist = 0;
        double yaw = 0;
        double pitch = 0;

        if(pipeline.hasTargets()){
            yaw = getYaw(pipeline.getBestTarget());
            pitch = getPitch(pipeline.getBestTarget());
            dist = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, 
            TARGET_HEIGHT_METERS, Units.degreesToRadians(CAMERA_YAW), Units.degreesToRadians(pitch));   
        }

        return (dist) / Math.sin(yaw);
    }

    @Override
    public void periodic(){
        pipeline = this.getLatestPipeline();
        // SmartDashboard.putNumber("forward distance to target", getX());
        // SmartDashboard.putNumber("horizontal distance to target", getY());
        // SmartDashboard.putNumber("vertical angle to target", getPitch(pipeline.getBestTarget()));
        // SmartDashboard.putNumber("horizontal angle to target", getYaw(pipeline.getBestTarget()));
    }

}
