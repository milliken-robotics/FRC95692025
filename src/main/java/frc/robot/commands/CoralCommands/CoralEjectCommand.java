package frc.robot.commands.CoralCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralEndeffactorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class CoralEjectCommand extends Command{
      private final CoralEndeffactorSubsystem coralEndeffactorSubsystem; 
      private final ElevatorSubsystem elevatorSubsystem; 


      private int level;   

    public CoralEjectCommand(CoralEndeffactorSubsystem coralEndeffactorSubsystem, ElevatorSubsystem elevatorSubsystem){
        this.coralEndeffactorSubsystem = coralEndeffactorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem; 
        addRequirements(coralEndeffactorSubsystem);

    }
    
      @Override
    public void initialize(){
        level = elevatorSubsystem.getLevel();
    }

    @Override
    public void execute(){
        if(level == 1 || level ==0){
            coralEndeffactorSubsystem.setVoltDiff(4);
        }
        else{
            coralEndeffactorSubsystem.setVolt(8);
        }
    }
    @Override
    public void end(boolean interrupted){
        new WaitCommand(0.5);
        coralEndeffactorSubsystem.stop();
    }
    @Override
    public boolean isFinished(){
        return coralEndeffactorSubsystem.beamBroken2();// Math.abs(900 - elevatorSubsystem.getPoint()) < 20; 
    }
}
