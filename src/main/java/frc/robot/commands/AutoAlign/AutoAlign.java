package frc.robot.commands.AutoAlign;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

import java.io.File;
import java.util.HashMap;
import java.util.Map;


import org.photonvision.targeting.PhotonPipelineResult;

public class AutoAlign extends Command{
    private final SwerveSubsystem swerveSubsystem; 
    // private final Field2d targetFieldAprilTag = new Field2d();
    private final Map<Integer, Pose2d> coordinates = new HashMap<>();
    private Integer aprilTag = 0;
    private final String side;
    Command alignCommand;
    

    public AutoAlign (String side, SwerveSubsystem swerveSubsystem, int aprilTag){
        this.swerveSubsystem = swerveSubsystem;
        this.side = side;
        this.aprilTag = aprilTag;
        getMap();
        addRequirements(swerveSubsystem);
        
    }

    @Override
    public void initialize(){
        // swerveSubsystem.driveToPose(alignedPose);
        // SmartDashboard.putData("targetFieldAprilTag", targetFieldAprilTag);
    }

    @Override
    public void execute(){

        //SmartDashboard.putNumber("April Tag from AutoAlign", aprilTag);
        alignCommand = swerveSubsystem.driveToPose(coordinates.get(aprilTag));
            alignCommand.schedule();

        // if(aprilTag!=null && aprilTag != 0 && coordinates.keySet().contains(aprilTag)) {
        //     alignCommand = swerveSubsystem.driveToPose(coordinates.get(aprilTag));
        //     alignCommand.schedule();
        // }
        // targetFieldAprilTag.setRobotPose(coordinates.get(aprilTag));

    }

    public boolean inKeysetXXX() {
        return coordinates.keySet().contains(aprilTag);
    }

    @Override
    public void end(boolean interrupted){
        alignCommand.cancel();
    }
    @Override
    public boolean isFinished(){
        return false;
    }

    public void getMap() {
        // if (this.side == "right") {
        //     File directory = new File(Filesystem.getDeployDirectory(),"rightautoalign.json");
        // } else if (this.side == "left") {
        //     File directory = new File(Filesystem.getDeployDirectory(),"leftautoalign.json");
        // }

        // // try (JsonReader reader = new JsonReader(fileReader)) {
        // //     }
        

        if (this.side == "left") {
            coordinates.put(6, new Pose2d(13.65, 2.8, new Rotation2d(5.24)));
            coordinates.put(7, new Pose2d(14.38, 3.91, new Rotation2d(0)));
            coordinates.put(8, new Pose2d(13.84, 5.14, new Rotation2d(1.05)));
            coordinates.put(9, new Pose2d(12.54, 5.2, new Rotation2d(2.09)));
            coordinates.put(10, new Pose2d(11.74, 4.14, new Rotation2d(3.14)));
            coordinates.put(11, new Pose2d(12.30, 2.98, new Rotation2d(4.19)));
            coordinates.put(17, new Pose2d(3.69, 2.96, new Rotation2d(4.19)));
            coordinates.put(18, new Pose2d(3.16, 4.18, new Rotation2d(3.14)));
            coordinates.put(19, new Pose2d(3.93, 5.24, new Rotation2d(2.09)));
            coordinates.put(20, new Pose2d(5.28, 5.07, new Rotation2d(1.05)));
            coordinates.put(21, new Pose2d(5.81, 3.95, new Rotation2d(0)));
            coordinates.put(22, new Pose2d(5.01, 2.8, new Rotation2d(5.24)));
        } else if (this.side == "right") {
            coordinates.put(6, new Pose2d(13.90, 2.96, new Rotation2d(5.24)));
            coordinates.put(7, new Pose2d(14.38, 4.18, new Rotation2d(0)));
            coordinates.put(8, new Pose2d(13.57, 5.24, new Rotation2d(1.05)));
            coordinates.put(9, new Pose2d(12.26, 5.07, new Rotation2d(2.09)));
            coordinates.put(10, new Pose2d(11.74, 3.87, new Rotation2d(3.14)));
            coordinates.put(11, new Pose2d(12.56, 2.8, new Rotation2d(4.19)));
            coordinates.put(17, new Pose2d(4.02, 2.81, new Rotation2d(4.19)));
            coordinates.put(18, new Pose2d(3.16, 3.87, new Rotation2d(3.14)));
            coordinates.put(19, new Pose2d(3.69, 5.10, new Rotation2d(2.09)));
            coordinates.put(20, new Pose2d(4.99, 5.24, new Rotation2d(1.05)));
            coordinates.put(21, new Pose2d(5.81, 4.18, new Rotation2d(0)));
            coordinates.put(22, new Pose2d(5.36, 2.97, new Rotation2d(5.24)));
        }
    } 

}
