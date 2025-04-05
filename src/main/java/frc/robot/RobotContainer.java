// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoAlignCoordinates;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FlashBang;
import frc.robot.commands.AlgaeCommands.AlgaeEject;
import frc.robot.commands.AlgaeCommands.AlgaeIntake;
import frc.robot.commands.AlgaeCommands.AlgaePivotDown;
import frc.robot.commands.AlgaeCommands.AlgaePivotMiddle;
import frc.robot.commands.AlgaeCommands.AlgaePivotUp;
import frc.robot.commands.AlgaeCommands.AlgaePivotUpper;
import frc.robot.commands.AlgaeCommands.AlgaeSeqIntake;
import frc.robot.commands.AutoAlign.AutoAlign;
import frc.robot.commands.CoralCommands.CoralEjectCommand;
import frc.robot.commands.CoralCommands.CoralForceDiffEjectCommand;
import frc.robot.commands.CoralCommands.CoralIntakeCommand;
import frc.robot.commands.ElevatorCommands.ElevatorCycle;
import frc.robot.commands.ElevatorCommands.ElevatorL1Command;
import frc.robot.commands.ElevatorCommands.ElevatorL2Command;
import frc.robot.commands.ElevatorCommands.ElevatorL3Command;
import frc.robot.commands.ElevatorCommands.ElevatorZeroCommand;
import frc.robot.subsystems.AlgaeEndeffactorSubsystem;
import frc.robot.subsystems.CoralEndeffactorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem driveBase = new SwerveSubsystem(); 
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(); 
  private final CoralEndeffactorSubsystem coralEndeffactorSubsystem = new CoralEndeffactorSubsystem(); 
  private final AlgaeEndeffactorSubsystem algaeEndeffactorSubsystem = new AlgaeEndeffactorSubsystem(); 
  private final LEDSubsystem ledSubsystem = new LEDSubsystem(); 
  private final Vision vision = new Vision();
  private final Map<Integer, Pose2d> coordinates = new HashMap<>();

  private final CommandPS5Controller controllerP = new CommandPS5Controller(OperatorConstants.kDriverControllerPort);
  
  private final CommandXboxController controllerX = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser;
  private double targetX;
  private double targetY;
  private double targetTheta;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    
    for(int i = 1; i<=22; i++){
      if(i==1){
        poseChooser.setDefaultOption(i+"", i);
      }
      else{
        poseChooser.addOption(i+"", i);
      }
    }
    SmartDashboard.putData("pose chooser", poseChooser);

    configureBindings();
    //driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    NamedCommands.registerCommand("EC", new CoralForceDiffEjectCommand(coralEndeffactorSubsystem, elevatorSubsystem, ledSubsystem));
    NamedCommands.registerCommand("IC", new CoralIntakeCommand(coralEndeffactorSubsystem, ledSubsystem));
    NamedCommands.registerCommand("L0", new ElevatorZeroCommand(elevatorSubsystem));
    NamedCommands.registerCommand("L1", new ElevatorL1Command(elevatorSubsystem));
    NamedCommands.registerCommand("L2", new ElevatorL2Command(elevatorSubsystem));
    NamedCommands.registerCommand("L3", new ElevatorL3Command(elevatorSubsystem));
    new ElevatorZeroCommand(elevatorSubsystem);

    autoChooser = AutoBuilder.buildAutoChooser();
    targetX = 0; 
    targetY = 0;
    targetTheta = 0; 


    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putNumber("X Autoalign", targetX);

    SmartDashboard.putNumber("Y Autoalign", targetY);
    SmartDashboard.putNumber("Theta Autoalign", targetTheta);

  }

  // public Pose2d getPose(int side) {
  //   int bestTag = vision.targetId;

  //   if (side == 1) {

  //   }
  // }



  public Pose2d getSelectedPose(){
    SmartDashboard.putString("chosen pose", ""+poseChooser.getSelected());
    //System.out.println(poseChooser.getSelected());

    if (poseChooser.getSelected().equals(6)) {
      return AutoAlignCoordinates.R.pose6;
  } else if (poseChooser.getSelected().equals(7)) {
      return AutoAlignCoordinates.R.pose7;
  } else if (poseChooser.getSelected().equals(8)) {
      return AutoAlignCoordinates.R.pose8;
  } else if (poseChooser.getSelected().equals(9)) {
      return AutoAlignCoordinates.R.pose9;
  } else if (poseChooser.getSelected().equals(10)) {
      return AutoAlignCoordinates.R.pose10;
  } else if (poseChooser.getSelected().equals(11)) {
      return AutoAlignCoordinates.R.pose11;
  } else if (poseChooser.getSelected().equals(17)) {
      return AutoAlignCoordinates.R.pose17;
  } else if (poseChooser.getSelected().equals(18)) {
      return AutoAlignCoordinates.R.pose18;
  } else if (poseChooser.getSelected().equals(19)) {
      return AutoAlignCoordinates.R.pose19;
  } else if (poseChooser.getSelected().equals(20)) {
      return AutoAlignCoordinates.R.pose20;
  } else if (poseChooser.getSelected().equals(21)) {
      return AutoAlignCoordinates.R.pose21;
  } else if (poseChooser.getSelected().equals(22)) {
      return AutoAlignCoordinates.R.pose22;
  } else {
      return AutoAlignCoordinates.R.pose22;  // default case
  }
   
    }
  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                                                ()-> controllerP.getLeftY()*-1,
                                                                ()-> controllerP.getLeftX()*-1)
                                                                .withControllerRotationAxis(controllerP::getRightX)
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(1)
                                                                .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(controllerP::getRightX, 
                                                                                            controllerP::getRightY)
                                                                                            .headingWhile(true);

  Command driveFieldOritentedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);   
  Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity); 
  
  
  
  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                                                        () -> -controllerP.getLeftY(),
                                                                        () -> -controllerP.getLeftX())
                                                                    .withControllerRotationAxis(() -> controllerP.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                controllerP.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                controllerP.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

 Command driveFieldOrientedDirectAngleKeyboard      = driveBase.driveFieldOriented(driveDirectAngleKeyboard);
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

   private final ElevatorZeroCommand zeroCommand = new ElevatorZeroCommand(elevatorSubsystem);
  private final ElevatorL2Command l2Command = new ElevatorL2Command(elevatorSubsystem);
  private final ElevatorL3Command l3Command = new ElevatorL3Command(elevatorSubsystem);
  private final Command[] elevatorCommands = { zeroCommand, l2Command, l3Command };
  private int curLevel = 0;
  private int currentAprilTag = 0;
  private final SendableChooser<Integer> poseChooser = new SendableChooser<>();

  private void scheduleElevatorCommandByLevel() {
    switch (elevatorSubsystem.level) {
      case 0:
        zeroCommand.schedule();
        break;
      case 1:
        l2Command.schedule();
        break;
      case 2:
        l3Command.schedule();
        break;
      default:
        // fallback
        zeroCommand.schedule();
        break;
    }
  }

  double x = 0; 
  double y = 0; 
  double theta = 0; 

  // public Pose2d getTargetPoseFromDashboard() {
  //   x = SmartDashboard.getNumber("X Autoalign", 0.0);
  //   y = SmartDashboard.getNumber("Y Autoalign", 0.0);
  //   theta = SmartDashboard.getNumber("Theta Autoalign", 0.0);
  //   return new Pose2d(x, y, Rotation2d.fromDegrees(theta));
  // }

  private void configureBindings() {
    
    // controller.L1().whileTrue(new RunCommand(() -> elevatorSubsystem.setVolt(5))).onFalse(new RunCommand (()->elevatorSubsystem.stop()));
    // controller.R1().whileTrue(new RunCommand(() -> elevatorSubsystem.setVolt(-5))).onFalse(new RunCommand (()->elevatorSubsystem.stop()));

    // controllerX.leftBumper().whileTrue(new RunCommand(() -> algaeEndeffactorSubsystem.setVolt(5))).onFalse(new RunCommand (()->algaeEndeffactorSubsystem.algaePivotStop()));
    // controllerX.rightBumper().whileTrue(new RunCommand(() -> algaeEndeffactorSubsystem.setVolt(-5))).onFalse(new RunCommand (()->algaeEndeffactorSubsystem.algaePivotStop()));


     if (RobotBase.isSimulation())
    {
      driveBase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    // controllerP.circle().onTrue(new ElevatorZeroCommand(elevatorSubsystem));
    // controllerP.cross().onTrue(new ElevatorL3Command(elevatorSubsystem));
    // controllerP.triangle().onTrue(new ElevatorL2Command(elevatorSubsystem));


    controllerP.R1().onTrue(new ElevatorCycle(elevatorSubsystem, 1));
    controllerP.L1().onTrue(new ElevatorCycle(elevatorSubsystem, -1));

    controllerX.leftBumper().onTrue(new CoralIntakeCommand(coralEndeffactorSubsystem,ledSubsystem));
    controllerX.rightBumper().onTrue(new CoralEjectCommand(coralEndeffactorSubsystem, elevatorSubsystem, ledSubsystem));

    controllerP.povUp().onTrue(Commands.runOnce(()-> driveBase.zeroGyro()));
    //currentAprilTag = vision.targetId;
    // controllerP.povLeft().whileTrue(new AutoAlign(driveBase, "left", currentAprilTag));
    // controllerP.povRight().whileTrue(new AutoAlign(driveBase, "right", currentAprilTag));
    // // .onFalse(
    // //   new InstantCommand((this.cancel()))
    // // );
    // controllerP.povRight().whileTrue(new AutoAlign("right", driveBase, currentAprilTag));


    controllerP.povDown().whileTrue(new FlashBang(ledSubsystem));   


    controllerX.y().onTrue(new AlgaePivotUp(algaeEndeffactorSubsystem));
    controllerX.x().onTrue(new AlgaePivotMiddle(algaeEndeffactorSubsystem));
    controllerX.a().onTrue(new AlgaePivotDown(algaeEndeffactorSubsystem));
    controllerX.b().onTrue(new AlgaePivotUpper(algaeEndeffactorSubsystem));

    controllerX.rightTrigger().whileTrue(new AlgaeIntake(algaeEndeffactorSubsystem));
    controllerX.leftTrigger().whileTrue(new AlgaeEject(algaeEndeffactorSubsystem));

    controllerX.povLeft().onTrue(Commands.runOnce(()-> coralEndeffactorSubsystem.diffRight()));
    controllerX.povRight().onTrue(Commands.runOnce(()-> coralEndeffactorSubsystem.diffLeft()));
  }

  // @Override
  // public void periodic() {

  // }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return driveBase.getAutonomousCommand("New Auto");
  // }
  
  // public void getMap(String side) {
    
  //   // if (this.side == "right") {
  //   //     File directory = new File(Filesystem.getDeployDirectory(),"rightautoalign.json");
  //   // } else if (this.side == "left") {
  //   //     File directory = new File(Filesystem.getDeployDirectory(),"leftautoalign.json");
  //   // }

  //   // // try (JsonReader reader = new JsonReader(fileReader)) {
  //   // //     }

  //   if (side == "left") {
  //       coordinates.put(6, new Pose2d(13.562, 2.828, new Rotation2d(Math.toRadians(300))));
  //       coordinates.put(7, new Pose2d(14.348, 3.191, new Rotation2d(0)));
  //       coordinates.put(8, new Pose2d(13.562, 5.224, new Rotation2d(Math.toRadians(60))));
  //       coordinates.put(9, new Pose2d(12.273, 5.059, new Rotation2d(Math.toRadians(120))));
  //       coordinates.put(10, new Pose2d(11.77, 4.191, new Rotation2d(Math.toRadians(180))));
  //       coordinates.put(11, new Pose2d(12.273, 2.993, new Rotation2d(Math.toRadians(240))));
  //       coordinates.put(17, new Pose2d(3.703, 2.993, new Rotation2d(Math.toRadians(240))));
  //       coordinates.put(18, new Pose2d(3.2, 4.191, new Rotation2d(Math.toRadians(180))));
  //       coordinates.put(19, new Pose2d(3.703, 5.059, new Rotation2d(Math.toRadians(120))));
  //       coordinates.put(20, new Pose2d(4.991, 5.224, new Rotation2d(Math.toRadians(60))));
  //       coordinates.put(21, new Pose2d(5.777, 4.191, new Rotation2d(0)));
  //       coordinates.put(22, new Pose2d(4.991, 2.828, new Rotation2d(Math.toRadians(300))));
  //   } else if (side == "right") {
  //       coordinates.put(6, new Pose2d(13.848, 2.993, new Rotation2d(Math.toRadians(300))));
  //       coordinates.put(7, new Pose2d(14.348, 4.861, new Rotation2d(0)));
  //       coordinates.put(8, new Pose2d(13.848, 5.059, new Rotation2d(Math.toRadians(60))));
  //       coordinates.put(9, new Pose2d(12.559, 5.224, new Rotation2d(Math.toRadians(120))));
  //       coordinates.put(10, new Pose2d(11.77, 3.861, new Rotation2d(Math.toRadians(180))));
  //       coordinates.put(11, new Pose2d(12.559, 2.828, new Rotation2d(Math.toRadians(240))));
  //       coordinates.put(17, new Pose2d(3.989, 2.828, new Rotation2d(Math.toRadians(240))));
  //       coordinates.put(18, new Pose2d(3.2, 3.861, new Rotation2d(Math.toRadians(180))));//
  //       coordinates.put(19, new Pose2d(3.989, 5.224, new Rotation2d(Math.toRadians(120))));
  //       coordinates.put(20, new Pose2d(5.277, 5.059, new Rotation2d(Math.toRadians(60))));
  //       coordinates.put(21, new Pose2d(5.777, 3.861, new Rotation2d(0)));
  //       coordinates.put(22, new Pose2d(5.277, 2.993, new Rotation2d(Math.toRadians(300))));
  //   }

  //   public void getPose(int aprilTag) {
    //  getMap();
  //     if (coordinates.keySet().contains(aprilTag)) {
    //     return coordinates.get(aprilTag);
    //     alignCommand = swerveSubsystem.driveToPose(coordinates.get(aprilTag));
  //   } 
  //   else {
  //       alignCommand = new InstantCommand(() -> {});
  //   }

  //   alignCommand.schedule();
  //   }
     
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // public void stopAutoBuilder() {
  //   if ()
}
