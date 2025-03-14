package frc.robot.commands.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class AlignToRight extends Command{
    private final SwerveSubsystem swerveSubsystem; 
    private final Pose2d alignedPose = new Pose2d(3.1, 4, new Rotation2d(135));
    private final Map<Integer, Pose2d> coordinates;

    public AlignToRight(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        coordinates = new HashMap<>();
        coordinates.put(17, new Pose2d(4.02, 2.81, new Rotation2d(4.19)));
        coordinates.put(18, new Pose2d(3.16, 3.87, new Rotation2d(3.14)));
        coordinates.put(19, new Pose2d(3.69, 5.10, new Rotation2d(2.09)));
        coordinates.put(20, new Pose2d(4.99, 5.24, new Rotation2d(1.05)));
        coordinates.put(21, new Pose2d(5.81, 4.18, new Rotation2d(0)));
        coordinates.put(22, new Pose2d(5.30, 3, new Rotation2d(5.24)));
        coordinates.put(6, new Pose2d(13.90, 2.96, new Rotation2d(5.24)));
        coordinates.put(7, new Pose2d(14.38, 4.18, new Rotation2d(0)));
        coordinates.put(8, new Pose2d(13.57, 5.24, new Rotation2d(1.05)));
        coordinates.put(9, new Pose2d(12.26, 5.07, new Rotation2d(2.09)));
        coordinates.put(10, new Pose2d(11.74, 3.87, new Rotation2d(3.14)));
        coordinates.put(11, new Pose2d(12.56, 2.8, new Rotation2d(4.19)));
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
        // swerveSubsystem.driveToPose(alignedPose);

    }

    public Pose2d getCoords(int aprilTag) {
        return coordinates.get(aprilTag);
    }

    @Override
    public void execute(){
       Command alignCommand = swerveSubsystem.driveToPose(coordinates.get(22));
       alignCommand.schedule();

    }
    @Override
    public void end(boolean interrupted){
        //swerveSubsystem.
    }
    @Override
    public boolean isFinished(){
        return false;
    }


}
