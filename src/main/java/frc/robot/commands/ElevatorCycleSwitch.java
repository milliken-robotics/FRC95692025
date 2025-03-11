package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstnats;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCycleSwitch extends Command{
    private final ElevatorSubsystem elevatorSubsystem; 
    private final double dir; 

    public ElevatorCycleSwitch(ElevatorSubsystem elevatorSubsystem, double dir){
        this.elevatorSubsystem = elevatorSubsystem;
        this.dir = dir; 
        addRequirements(elevatorSubsystem);

    }
    
      @Override
    public void initialize(){

        int curlevel = elevatorSubsystem.getLevel();
        curlevel+= dir; 
        if (curlevel==4) {
            curlevel=0; 
        }

        elevatorSubsystem.setLevel(curlevel);
    }

    @Override
    public void execute(){       
    }
    @Override
    public void end(boolean interrupted){
    }
    @Override
    public boolean isFinished(){
        return true;// Math.abs(900 - elevatorSubsystem.getPoint()) < 20; 
    }
    
}
