package frc.robot.commands.AutoAlign;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

import java.io.File;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.opencv.core.Algorithm;
import org.photonvision.targeting.PhotonPipelineResult;

public class AutoAlign extends Command{
    private final SwerveSubsystem swerveSubsystem; 
    // private final Field2d targetFieldAprilTag = new Field2d();
    // private Integer aprilTag = 0;
    // private final String side;
    //private final SendableChooser<Integer> s_aprilTag = new SendableChooser<>();
    //private final Pose2d driveTo; 
    Command alignCommand;

    

    public AutoAlign (SwerveSubsystem swerveSubsystem, Command alignCommand){
        this.swerveSubsystem = swerveSubsystem;
        this.alignCommand = alignCommand;
        //this.driveTo = pose;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
       // swerveSubsystem.driveToPose(alignedPose);
       // SmartDashboard.putData("targetFieldAprilTag", targetFieldAprilTag);
    //     s_aprilTag.setDefaultOption("Default", 1);
    //     for (int key: coordinates.keySet()) {
    //         s_aprilTag.addOption(Integer.toString(key), key);
    //     }

    //    SmartDashboard.putData("April tag selector", s_aprilTag);
    }

    // @Override
    // public void execute(){
    //     // //does all the smartboard stuff
    //     // SmartDashboard.putData("auto align to:", s_aprilTag);
    //     if (inKeysetXXX()) {
    //         alignCommand = swerveSubsystem.driveToPose(coordinates.get(aprilTag));
    //     } 
    //     else {
    //         alignCommand = new InstantCommand(() -> {});
    //     }

       // alignCommand.schedule();


    //     // if(aprilTag!=null && aprilTag != 0 && coordinates.keySet().contains(aprilTag)) {
    //     //     alignCommand = swerveSubsystem.driveToPose(coordinates.get(aprilTag));
    //     //     alignCommand.schedule();
    //     // }
    //     // targetFieldAprilTag.setRobotPose(coordinates.get(aprilTag));

    // }

    // public boolean inKeysetXXX() {
    //     return coordinates.keySet().contains(aprilTag);
    // }

    // @Override
    // public void end(boolean interrupted){
    //     alignCommand.cancel();
    // }
    // @Override
    // public boolean isFinished(){
    //     return false;
    // }



}
