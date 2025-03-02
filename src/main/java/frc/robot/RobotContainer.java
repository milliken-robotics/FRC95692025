// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeEject;
import frc.robot.commands.AlgaeIntake;
import frc.robot.commands.AlgaePivotDown;
import frc.robot.commands.AlgaePivotMiddle;
import frc.robot.commands.AlgaePivotUp;
import frc.robot.commands.AlgaePivotUpper;
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
import swervelib.SwerveInputStream;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

  private final CommandPS4Controller controllerP = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  
  private final CommandXboxController controllerX = new CommandXboxController(1);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings



    configureBindings();
    driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    NamedCommands.registerCommand("EjectCoral", new CoralEjectCommand(coralEndeffactorSubsystem));
    NamedCommands.registerCommand("IntakeCoral", new CoralIntakeCommand(coralEndeffactorSubsystem));

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

    controllerP.circle().onTrue(new ElevatorZeroCommand(elevatorSubsystem));
    controllerP.cross().onTrue(new ElevatorL3Command(elevatorSubsystem));
    controllerP.triangle().onTrue(new ElevatorL2Command(elevatorSubsystem));
    controllerX.leftBumper().onTrue(new CoralIntakeCommand(coralEndeffactorSubsystem));
    controllerX.rightBumper().onTrue(new CoralEjectCommand(coralEndeffactorSubsystem));

    controllerP.povLeft().onTrue(Commands.runOnce(()-> driveBase.resetHeading()));


    controllerX.y().onTrue(new AlgaePivotUp(algaeEndeffactorSubsystem));
    controllerX.x().onTrue(new AlgaePivotMiddle(algaeEndeffactorSubsystem));
    controllerX.a().onTrue(new AlgaePivotDown(algaeEndeffactorSubsystem));
    controllerX.b().onTrue(new AlgaePivotUpper(algaeEndeffactorSubsystem));

    controllerX.rightTrigger().onTrue(new AlgaeIntake(algaeEndeffactorSubsystem));
    controllerX.leftTrigger().whileTrue(new AlgaeEject(algaeEndeffactorSubsystem));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return driveBase.getAutonomousCommand("Blue Auto");
  }
}
