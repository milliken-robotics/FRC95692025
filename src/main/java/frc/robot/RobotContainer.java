// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CoralEjectCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.ElevatorL1Command;
import frc.robot.commands.ElevatorL2Command;
import frc.robot.commands.ElevatorL3Command;
import frc.robot.commands.ElevatorZeroCommand;
import frc.robot.subsystems.AlgaeEndeffactorSubsystem;
import frc.robot.subsystems.CoralEndeffactorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
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

  private final CommandPS4Controller controller = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    NamedCommands.registerCommand("test", Commands.print("hellow"));

  }
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                                                ()-> controller.getLeftY()*-1,
                                                                ()-> controller.getLeftX()*-1)
                                                                .withControllerRotationAxis(controller::getRightX)
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(1)
                                                                .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(controller::getRightX, 
                                                                                            controller::getRightY)
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
    controller.L1().whileTrue(new RunCommand(() -> elevatorSubsystem.setVolt(5))).onFalse(new RunCommand (()->elevatorSubsystem.stop()));
    controller.R1().whileTrue(new RunCommand(() -> elevatorSubsystem.setVolt(-5))).onFalse(new RunCommand (()->elevatorSubsystem.stop()));
    controller.circle().onTrue(new ElevatorZeroCommand(elevatorSubsystem));
    controller.cross().onTrue(new ElevatorL3Command(elevatorSubsystem));
    controller.triangle().onTrue(new ElevatorL2Command(elevatorSubsystem));
    controller.povDown().onTrue(new CoralIntakeCommand(coralEndeffactorSubsystem));
    controller.povUp().onTrue(new CoralEjectCommand(coralEndeffactorSubsystem));
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return driveBase.getAutonomousCommand("New Auto");
  }
}
