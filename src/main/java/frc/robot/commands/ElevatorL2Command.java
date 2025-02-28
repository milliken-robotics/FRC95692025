package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstnats;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorL2Command extends Command {
    private final ElevatorSubsystem elevatorSubsystem; 

    public ElevatorL2Command(ElevatorSubsystem elevatorSubsystem){
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);

    }
    
      @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        elevatorSubsystem.setPoint(ElevatorConstnats.L2);
       
    }
    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.stop();
    }
    @Override
    public boolean isFinished(){
        return false;// Math.abs(900 - elevatorSubsystem.getPoint()) < 20; 
    }
    
}
