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
    private PhotonCamera endEffectorCameraR = new PhotonCamera("End Effector Camera"); 
    private PhotonCamera endEffectorCameraL = new PhotonCamera("left camera");
    
    //stored value of robto cam postiont
    private Transform3d robotToCamR = new Transform3d(new Translation3d(-0.1317, 1.317, 0.338), new Rotation3d(-90, 0, 135));
    private Transform3d robotToCamL = new Transform3d(new Translation3d(-0.1317, 0, 0.338),
            new Rotation3d(-90, 0, 135));

    //x: 0.327
    //y: 0.1317
    //z: 0.338
    
    //calculator for pose
    private PhotonPoseEstimator photonPoseEstimatorR = new PhotonPoseEstimator(
        fieldLayout, 
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCamR);
    private PhotonPoseEstimator photonPoseEstimatorL = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCamL);

    //this is just for smart dashboard purposes
    private final Field2d field = new Field2d();

    private List<PhotonTrackedTarget> currentAprilTags; 
    // private Optional<Pose3d> tagPose3dOpt;
    // public double xTag = 0;
    // public double yTag = 0;;
    // public double rotationTag = 0;

    //last reference point
    private Pose3d referencePose; 

    //estimated pose
    private Optional<EstimatedRobotPose> latestEstimatedPoseR = Optional.empty();
    private Optional<EstimatedRobotPose> latestEstimatedPoseL = Optional.empty();

    Pose3d estimatedRobotPoseR =  new Pose3d(0,0,0,new Rotation3d(0,0,0));
    Pose3d estimatedRobotPoseL = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    
    //construct stuff
    public Vision(){
        SmartDashboard.putData("photon subsytem", field);
        referencePose = new Pose3d(0,0,0,new Rotation3d(0,0,0));
        
    }

    public Optional<EstimatedRobotPose> getR() {
        return latestEstimatedPoseR;
    }
    
    public Optional<EstimatedRobotPose> getL() {
        return latestEstimatedPoseL;
    }
    

    //get pose 3d 
    public Optional<Pose3d> getPose3dR() {
        return latestEstimatedPoseR.map(stuff -> stuff.estimatedPose);
    }
    
    public Optional<Pose3d> getPose3dL() {
        return latestEstimatedPoseL.map(stuff -> stuff.estimatedPose);
    }

    //get pose 2d (to who ever is reading my code, use this for odom) 
    public Optional<Pose2d> getPose2dR(){
        return getPose3dR().map(pose3d -> pose3d.toPose2d());
    }

    public Optional<Pose2d> getPose2dL() {
        return getPose3dL().map(pose3d -> pose3d.toPose2d());
    }

    public List<PhotonTrackedTarget> getAprilTags(){
        return currentAprilTags;
    }
    public void setReferencePose (Pose3d refPose3d){
        referencePose = refPose3d;
    }

    public Optional<Double> getTimestampR() {
        return latestEstimatedPoseR.map(x -> x.timestampSeconds);
    }

    public Optional<Double> getTimestampL() {
        return latestEstimatedPoseL.map(x -> x.timestampSeconds);
    }
    //update the pose every so often and also set the refrence point 
    @Override
    public void periodic(){


        var resultR = endEffectorCameraR.getLatestResult();
        boolean hasTargetsR = resultR.hasTargets();
        currentAprilTags = resultR.getTargets();

        var resultL = endEffectorCameraL.getLatestResult();
        boolean hasTargetsL= resultL.hasTargets();
        currentAprilTags = resultL.getTargets();

        if(hasTargetsR){
            latestEstimatedPoseR = photonPoseEstimatorR.update(resultR);
        }
        if (hasTargetsL) {
            latestEstimatedPoseL = photonPoseEstimatorL.update(resultL);
        }
        photonPoseEstimatorR.setReferencePose(referencePose);
        SmartDashboard.putBoolean("has target R", hasTargetsR);
        SmartDashboard.putBoolean("has target L", hasTargetsL);

        List<PhotonTrackedTarget> targets;
        PhotonTrackedTarget bestTarget;
        int targetId =0; 
        

        if(hasTargetsR){
            targets = resultR.getTargets();
            bestTarget = resultR.getBestTarget();
            targetId = bestTarget.fiducialId;

            if(fieldLayout.getTagPose(bestTarget.getFiducialId()).isPresent()){
                estimatedRobotPoseR = PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(), fieldLayout.getTagPose(bestTarget.getFiducialId()).get(), bestTarget.getBestCameraToTarget());
                Pose2d robotconversion = new Pose2d(estimatedRobotPoseR.getTranslation().getX(), estimatedRobotPoseL.getTranslation().getY(), new Rotation2d(estimatedRobotPoseL.getRotation().getZ()));
                field.setRobotPose(robotconversion);
                // tagPose3dOpt = fieldLayout.getTagPose(targetId);
                
            }


        }
        SmartDashboard.putNumber("AprilTag X", fieldLayout.getTagPose(18).get().getX());
        SmartDashboard.putNumber("AprilTag Y", fieldLayout.getTagPose(18).get().getY());
        SmartDashboard.putNumber("AprilTag Rotation (deg)", fieldLayout.getTagPose(18).get().getRotation().getAngle());
        
       
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
