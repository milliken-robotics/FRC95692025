package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeEndeffactorSubsystem;
import frc.robot.subsystems.CoralEndeffactorSubsystem;

public class AlgaePivotUpper extends Command {

      private final AlgaeEndeffactorSubsystem algaeEndeffactorSubsystem; 
      private final Timer time = new Timer();

    public AlgaePivotUpper(AlgaeEndeffactorSubsystem algaeEndeffactorSubsystem){
        this.algaeEndeffactorSubsystem = algaeEndeffactorSubsystem;
        addRequirements(algaeEndeffactorSubsystem);
    }
    
      @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        algaeEndeffactorSubsystem.algaeSetPoint(1.6);
       
    }
    @Override
    public void end(boolean interrupted){
        algaeEndeffactorSubsystem.algaePivotStop();
    }
    @Override
    public boolean isFinished(){
        return false;// Math.abs(900 - elevatorSubsystem.getPoint()) < 20; 
    }
    
}
