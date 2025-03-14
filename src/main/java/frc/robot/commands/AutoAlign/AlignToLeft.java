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

public class AlignToLeft extends Command{
    private final SwerveSubsystem swerveSubsystem; 
    private final Pose2d alignedPose = new Pose2d(3.1, 4, new Rotation2d(135));
    private final Map<Integer, Pose2d> coordinates;

    public AlignToLeft (SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        coordinates = new HashMap<>();
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
