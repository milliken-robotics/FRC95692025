package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;



public class SwerveSubsystem extends SubsystemBase{

    Vision vision = new Vision();
    private final Field2d field = new Field2d();
    private final Field2d photonfield2d = new Field2d();

    File directory = new File(Filesystem.getDeployDirectory(),"swerve");
    SwerveDrive swerveDrive;

    public SwerveSubsystem(){
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        SmartDashboard.putData("swerve + photon field", field);
        SmartDashboard.putData("photon field 2d", photonfield2d);
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
            swerveDrive.setHeadingCorrection(false);
            swerveDrive.setCosineCompensator(false);
            setupPathPlanner();
            RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
    }

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
                    ChassisSpeeds correctedSpeeds = new ChassisSpeeds(
                        speedsRobotRelative.vxMetersPerSecond, 
                        speedsRobotRelative.vyMetersPerSecond,
                        -speedsRobotRelative.omegaRadiansPerSecond
                    );

                if (enableFeedforward)
                {
                    swerveDrive.drive(
                        speedsRobotRelative,
                        swerveDrive.kinematics.toSwerveModuleStates(correctedSpeeds),
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
                    new PIDConstants(4.79645, 0.0, 0.0),
                    // Translation PID constants
                    new PIDConstants(5, 0.0, 0.0) //0.02
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
        SmartDashboard.putNumber("Module 1", swerveDrive.getModules()[0].getAbsolutePosition());
        SmartDashboard.putNumber("Module 2", swerveDrive.getModules()[1].getAbsolutePosition());
        SmartDashboard.putNumber("Module 3", swerveDrive.getModules()[2].getAbsolutePosition());
        SmartDashboard.putNumber("Module 4", swerveDrive.getModules()[3].getAbsolutePosition());
        // SmartDashboard.putNumber("heading", getHeading().getDegrees());

        Optional<Pose2d> visionPoseOptR = vision.getPose2dR();
        Optional<Double> timestampOptR = vision.getTimestampR();

        Optional<Pose2d> visionPoseOptL = vision.getPose2dL();
        Optional<Double> timestampOptL = vision.getTimestampL();
        boolean isTherePhotonR = false; 
        boolean isTherePhotonL = false;
      
        
        if (visionPoseOptR.isPresent()) {
            swerveDrive.addVisionMeasurement(visionPoseOptR.get(), timestampOptR.get());
            photonfield2d.setRobotPose(visionPoseOptR.get());
            isTherePhotonR = true;
        //      SmartDashboard.putNumber("photon Robot X", visionPoseOpt.get().getX());
        // SmartDashboard.putNumber("photon Robot Y", visionPoseOpt.get().getY());
        // SmartDashboard.putNumber("photon Robot Heading (deg)",visionPoseOpt.get().getRotation().getDegrees());
        }
        else{
            isTherePhotonR = false;
        }

        if (visionPoseOptL.isPresent()) {
            swerveDrive.addVisionMeasurement(visionPoseOptL.get(), timestampOptL.get());
            photonfield2d.setRobotPose(visionPoseOptL.get());
            isTherePhotonL = true;
            // SmartDashboard.putNumber("photon Robot X", visionPoseOpt.get().getX());
            // SmartDashboard.putNumber("photon Robot Y", visionPoseOpt.get().getY());
            // SmartDashboard.putNumber("photon Robot Heading
            // (deg)",visionPoseOpt.get().getRotation().getDegrees());
        } 
        else {
            isTherePhotonL = false;
        }
        // visionPoseOpt.ifPresent(est->{
        //     swerveDrive.addVisionMeasurement(visionPoseOpt.get(), timestampOpt.get());
        //     photonfield2d.setRobotPose(visionPoseOpt.get());
        // });

        Pose2d currentPose = swerveDrive.getPose();
        field.setRobotPose(currentPose);

        SmartDashboard.putNumber("Robot X", currentPose.getTranslation().getX());
        SmartDashboard.putNumber("Robot Y", currentPose.getTranslation().getY());
        SmartDashboard.putNumber("Robot Heading (deg)", currentPose.getRotation().getDegrees());
        SmartDashboard.putBoolean("is there photon R", isTherePhotonR);
        SmartDashboard.putBoolean("is there photon L", isTherePhotonL);
        // double xTag = vision.xTag;
        // double yTag = vision.yTag;
        // double rotationTag = vision.rotationTag;

       
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            zeroGyro();
            // Set the pose 180 degrees
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            zeroGyro();
        }
    }

    public void resetOdemWithVision(){
        Optional<Pose2d> visionPoseOpt = vision.getPose2dR();
        if (visionPoseOpt.isPresent()) {
            // Reset odometry wtih vis
            swerveDrive.resetOdometry(visionPoseOpt.get());
        }
    }

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }
    
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
        return run(() -> {
        // Make the robot move
        swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                                translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                            Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                            true,
                            false);
        });
    }

    // public Command driveToPose(Pose2d pose)
    // {
    // // constraint
    //     PathConstraints constraints = new PathConstraints(
    //         3, 4.0,
    //         8, Units.degreesToRadians(720));

    //     return AutoBuilder.pathfindToPose(
    //         pose,
    //         constraints,
    //         edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    //                                     );
    // }

       public Command driveToPose(Pose2d pose)
    {
    // constraint
        PathConstraints constraints = new PathConstraints(
            2, 2.0,
            10, Units.degreesToRadians(720));

        return AutoBuilder.pathfindToPose(
            pose,
            constraints,
            edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                        );
    }

    

    


}