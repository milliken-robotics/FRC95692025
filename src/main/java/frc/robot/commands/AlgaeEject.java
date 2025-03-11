package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeEndeffactorSubsystem;

public class AlgaeEject extends Command {
    
      private final AlgaeEndeffactorSubsystem algaeEndeffactorSubsystem; 
      private final Timer time = new Timer();

    public AlgaeEject(AlgaeEndeffactorSubsystem algaeEndeffactorSubsystem){
        this.algaeEndeffactorSubsystem = algaeEndeffactorSubsystem;
        //addRequirements(algaeEndeffactorSubsystem);
    }
    
      @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        algaeEndeffactorSubsystem.setSpeed(12);
       
    }
    @Override
    public void end(boolean interrupted){
        algaeEndeffactorSubsystem.setSpeed(0);
    }
    @Override
    public boolean isFinished(){
        return false;// Math.abs(900 - elevatorSubsystem.getPoint()) < 20; 
    }

}
