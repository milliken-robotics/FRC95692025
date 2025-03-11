package frc.robot.subsystems;

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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Vision extends SubsystemBase{
    //field 
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    
    //camera
    private PhotonCamera endEffectorCamera = new PhotonCamera("End Effector Camera"); 
    
    //stored value of robto cam postiont
    private Transform3d robotToCam = new Transform3d(new Translation3d(-4.75, -10.375, 11.375), new Rotation3d(-90, 0, 0));
    
    //calculator for pose
    private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
        fieldLayout, 
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCam);

    //this is just for smart dashboard purposes
    private final Field2d field = new Field2d();

    //last reference point
    private Pose3d referencePose; 

    //estimated pose
    private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();
    
    //construct stuff
    public Vision(){
        SmartDashboard.putData("field", field);
        referencePose = new Pose3d(0,0,0,new Rotation3d(0,0,0));
        
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose (){
        return latestEstimatedPose;
    }

    //update the pose every so often and also set the refrence point 
    @Override
    public void periodic(){


        var result = endEffectorCamera.getLatestResult();
        boolean hasTargets = result.hasTargets();

        if(hasTargets){
            latestEstimatedPose = photonPoseEstimator.update(result);
        }
        photonPoseEstimator.setReferencePose(referencePose);
        // SmartDashboard.putBoolean("has target", hasTargets);


        // List<PhotonTrackedTarget> targets;
        // PhotonTrackedTarget bestTarget;
        // int targetId =0; 
        
        // if(hasTargets){
        //     targets = result.getTargets();
        //     bestTarget = result.getBestTarget();
        //     targetId = bestTarget.fiducialId;

        //     if(fieldLayout.getTagPose(bestTarget.getFiducialId()).isPresent()){
        //         Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(), fieldLayout.getTagPose(bestTarget.getFiducialId()).get(), bestTarget.getBestCameraToTarget());
        //         Pose2d robotconversion = new Pose2d(robotPose.getTranslation().getX(), robotPose.getTranslation().getY(), new Rotation2d(robotPose.getRotation().getZ()));
        //         field.setRobotPose(robotconversion);
        //     }


        // } 
        
       
        // SmartDashboard.putNumber("id april", targetId);



    }
}
