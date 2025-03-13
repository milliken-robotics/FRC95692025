package frc.robot.commands.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignAprilTag18 extends Command{
    private final SwerveSubsystem swerveSubsystem; 
    private final Pose2d alignedPose = new Pose2d(3.1, 4, new Rotation2d(135));
    public AlignAprilTag18 (SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
        // swerveSubsystem.driveToPose(alignedPose);
    }

    @Override
    public void execute(){
       Command alignCommand = swerveSubsystem.driveToPose(alignedPose);
       alignCommand.schedule();
    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return false;
    }


}
