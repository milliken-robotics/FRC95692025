// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeEject;
import frc.robot.commands.AlgaePivotDown;
import frc.robot.commands.AlgaePivotMiddle;
import frc.robot.commands.AlgaePivotUp;
import frc.robot.commands.AlgaePivotUpper;
import frc.robot.commands.AlgaeSeqIntake;
import frc.robot.commands.CoralEjectCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.ElevatorL1Command;
import frc.robot.commands.ElevatorL2Command;
import frc.robot.commands.ElevatorL3Command;
import frc.robot.commands.ElevatorZeroCommand;
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

  private final CommandPS5Controller controllerP = new CommandPS5Controller(OperatorConstants.kDriverControllerPort);
  
  private final CommandXboxController controllerX = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    //driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    NamedCommands.registerCommand("EjectCoral", new CoralEjectCommand(coralEndeffactorSubsystem, elevatorSubsystem));
    NamedCommands.registerCommand("IntakeCoral", new CoralIntakeCommand(coralEndeffactorSubsystem));
    NamedCommands.registerCommand("L0", new ElevatorZeroCommand(elevatorSubsystem));
    NamedCommands.registerCommand("L1", new ElevatorL1Command(elevatorSubsystem));
    NamedCommands.registerCommand("L2", new ElevatorL2Command(elevatorSubsystem));
    NamedCommands.registerCommand("L3", new ElevatorL3Command(elevatorSubsystem));
    new ElevatorZeroCommand(elevatorSubsystem);

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
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
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
  private void configureBindings() {
    
    // controller.L1().whileTrue(new RunCommand(() -> elevatorSubsystem.setVolt(5))).onFalse(new RunCommand (()->elevatorSubsystem.stop()));
    // controller.R1().whileTrue(new RunCommand(() -> elevatorSubsystem.setVolt(-5))).onFalse(new RunCommand (()->elevatorSubsystem.stop()));

    // controllerX.L1().whileTrue(new RunCommand(() -> algaeEndeffactorSubsystem.setVolt(5))).onFalse(new RunCommand (()->algaeEndeffactorSubsystem.algaePivotStop()));
    // controllerX.R1().whileTrue(new RunCommand(() -> algaeEndeffactorSubsystem.setVolt(-5))).onFalse(new RunCommand (()->algaeEndeffactorSubsystem.algaePivotStop()));


     if (RobotBase.isSimulation())
    {
      driveBase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    controllerP.circle().onTrue(new ElevatorZeroCommand(elevatorSubsystem));
    controllerP.cross().onTrue(new ElevatorL3Command(elevatorSubsystem));
    controllerP.triangle().onTrue(new ElevatorL2Command(elevatorSubsystem));
    controllerX.leftBumper().onTrue(new CoralIntakeCommand(coralEndeffactorSubsystem));
    controllerX.rightBumper().onTrue(new CoralEjectCommand(coralEndeffactorSubsystem, elevatorSubsystem));

    controllerP.povLeft().onTrue(Commands.runOnce(()-> driveBase.zeroGyro()));


    controllerX.y().onTrue(new AlgaePivotUp(algaeEndeffactorSubsystem));
    controllerX.x().onTrue(new AlgaePivotMiddle(algaeEndeffactorSubsystem));
    controllerX.a().onTrue(new AlgaePivotDown(algaeEndeffactorSubsystem));
    controllerX.b().onTrue(new AlgaePivotUpper(algaeEndeffactorSubsystem));

    controllerX.rightTrigger().whileTrue(new AlgaeSeqIntake(algaeEndeffactorSubsystem).repeatedly());
    controllerX.leftTrigger().whileTrue(new AlgaeEject(algaeEndeffactorSubsystem));

    controllerX.povLeft().onTrue(Commands.runOnce(()-> coralEndeffactorSubsystem.diffRight()));
    controllerX.povRight().onTrue(Commands.runOnce(()-> coralEndeffactorSubsystem.diffLeft()));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return driveBase.getAutonomousCommand("New Auto");
  // }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
