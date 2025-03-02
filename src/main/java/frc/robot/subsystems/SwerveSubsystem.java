package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveModule;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.*;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase{

    

    File directory = new File(Filesystem.getDeployDirectory(),"swerve");
    SwerveDrive  swerveDrive;

    public SwerveSubsystem(){
         try
            {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED,
                                                                        new Pose2d(new Translation2d(Meter.of(1),
                                                                                                    Meter.of(4)),
                                                                                    Rotation2d.fromDegrees(0)));
            // Alternative method if you don't want to supply the conversion factor via JSON files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
            } catch (Exception e)
            {
            throw new RuntimeException(e);
            }  
            setupPathPlanner();
            RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::resetHeading));
    }

    // public void zeroGyroWithAlliance(){
    //     if(isRedAlliance()){
    //         resetHeading();
    //         resetOdemetry(new Pose2D(getPose().getTranslation(), Rotation2d.));
    //     }
    // }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        
        swerveDrive.driveFieldOriented(velocity);
        

    }
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
        return run(()->{
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }
    public void setupPathPlanner()
    {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try
        {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;
            // Configure AutoBuilder last
            AutoBuilder.configure(
                swerveDrive::getPose,
                // Robot pose supplier
                swerveDrive::resetOdometry,
                // Method to reset odometry (will be called if your auto has a starting pose)
                swerveDrive::getRobotVelocity,
                // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speedsRobotRelative, moduleFeedForwards) -> {
                if (enableFeedforward)
                {
                    swerveDrive.drive(
                        speedsRobotRelative,
                        swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                        moduleFeedForwards.linearForces()
                                    );
                } else
                {
                    swerveDrive.setChassisSpeeds(speedsRobotRelative);
                }
                },
                // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController(
                    // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.70645, 0.0, 0.0),
                    // Translation PID constants
                    new PIDConstants(0.012, 0.0, 0.0)
                    // Rotation PID constants
                ),
                config,
                // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent())
                {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this
                // Reference to this subsystem to set requirements
                                );

        } catch (Exception e)
        {
            // Handle exception as needed
            e.printStackTrace();
        }    
    }

    public Command getAutonomousCommand(String pathName)
    {
      // Create a path following command using AutoBuilder. This will also trigger event markers.
      return new PathPlannerAuto(pathName);
    }

    public void resetHeading(){
        swerveDrive.zeroGyro();
        
    }

    @Override
    public void periodic(){
        // SmartDashboard.putNumber("Module 1", swerveDrive.getModules()[0].getAbsolutePosition());
        // SmartDashboard.putNumber("Module 2", swerveDrive.getModules()[1].getAbsolutePosition());
        // SmartDashboard.putNumber("Module 3", swerveDrive.getModules()[2].getAbsolutePosition());
        // SmartDashboard.putNumber("Module 4", swerveDrive.getModules()[3].getAbsolutePosition());
    }
}
