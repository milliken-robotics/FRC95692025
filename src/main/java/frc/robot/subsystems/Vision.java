package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;


import java.lang.StackWalker.Option;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;



public class Vision extends SubsystemBase{
    //field 
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    
    //camera
    private PhotonCamera endEffectorCamera = new PhotonCamera("End Effector Camera"); 
    
    //stored value of robto cam postiont
    private Transform3d robotToCam = new Transform3d(new Translation3d(-0.1317, 1.317, 0.338), new Rotation3d(-90, 0, 135));

    //x: 0.327
    //y: 0.1317
    //z: 0.338
    
    //calculator for pose
    private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
        fieldLayout, 
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCam);

    //this is just for smart dashboard purposes
    private final Field2d field = new Field2d();

    private List<PhotonTrackedTarget> currentAprilTags; 
    private Optional<Pose3d> tagPose3dOpt;
    public double xTag = 0;
    public double yTag = 0;;
    public double rotationTag = 0;

    //last reference point
    private Pose3d referencePose; 

    //estimated pose
    private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();

    Pose3d estimatedRobotPose =  new Pose3d(0,0,0,new Rotation3d(0,0,0));
    
    //construct stuff
    public Vision(){
        SmartDashboard.putData("photon subsytem", field);
        referencePose = new Pose3d(0,0,0,new Rotation3d(0,0,0));
        
    }

    public Optional<EstimatedRobotPose> get(){
        return latestEstimatedPose;
    }

    //get pose 3d 
    public Optional<Pose3d> getPose3d() {
        return latestEstimatedPose.map(stuff -> stuff.estimatedPose);
    }

    //get pose 2d (to who ever is reading my code, use this for odom) 
    public Optional<Pose2d> getPose2d(){
        return getPose3d().map(pose3d -> pose3d.toPose2d());
    }

    public List<PhotonTrackedTarget> getAprilTags(){
        return currentAprilTags;
    }
    public void setReferencePose (Pose3d refPose3d){
        referencePose = refPose3d;
    }

    public Optional<Double> getTimestamp() {
        return latestEstimatedPose.map(x -> x.timestampSeconds);
    }

    //update the pose every so often and also set the refrence point 
    @Override
    public void periodic(){


        var result = endEffectorCamera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        currentAprilTags = result.getTargets();

        if(hasTargets){
            latestEstimatedPose = photonPoseEstimator.update(result);
        }
        photonPoseEstimator.setReferencePose(referencePose);
        SmartDashboard.putBoolean("has target", hasTargets);


        List<PhotonTrackedTarget> targets;
        PhotonTrackedTarget bestTarget;
        int targetId =0; 
        

        if(hasTargets){
            targets = result.getTargets();
            bestTarget = result.getBestTarget();
            targetId = bestTarget.fiducialId;

            if(fieldLayout.getTagPose(bestTarget.getFiducialId()).isPresent()){
                estimatedRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(), fieldLayout.getTagPose(bestTarget.getFiducialId()).get(), bestTarget.getBestCameraToTarget());
                Pose2d robotconversion = new Pose2d(estimatedRobotPose.getTranslation().getX(), estimatedRobotPose.getTranslation().getY(), new Rotation2d(estimatedRobotPose.getRotation().getZ()));
                field.setRobotPose(robotconversion);
                tagPose3dOpt = fieldLayout.getTagPose(targetId);
                
            }


        } 

     

        if (tagPose3dOpt.isPresent()) {
            // Convert the 3D pose to a 2D pose
            Pose2d tagPose2d = tagPose3dOpt.get().toPose2d();

            // Extract x, y, and rotation (in degrees)
            xTag = tagPose2d.getTranslation().getX();
            yTag = tagPose2d.getTranslation().getY();
            rotationTag = tagPose2d.getRotation().getDegrees();

            // For example, publish them to SmartDashboard
            
        }

        SmartDashboard.putNumber("AprilTag X", xTag);
        SmartDashboard.putNumber("AprilTag Y", yTag);
        SmartDashboard.putNumber("AprilTag Rotation (deg)", rotationTag);
        
       
        SmartDashboard.putNumber("id april", targetId);

    }

    // public Optional<Pose3d> getEstimatedGlobalPose (){

    // if(latestEstimatedPose.isPresent()){
    // //Pose3d pose = latestEstimatedPose.get();
    // }
    // else {

    // }
    // }

}
