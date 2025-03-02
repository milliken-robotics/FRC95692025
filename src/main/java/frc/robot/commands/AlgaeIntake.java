package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeEndeffactorSubsystem;

public class AlgaeIntake extends Command {
    
      private final AlgaeEndeffactorSubsystem algaeEndeffactorSubsystem; 
      private final Timer timer = new Timer();
      private final double pulse = 5;
      private boolean isMotor = false; 

    public AlgaeIntake(AlgaeEndeffactorSubsystem algaeEndeffactorSubsystem){
        this.algaeEndeffactorSubsystem = algaeEndeffactorSubsystem;
        addRequirements(algaeEndeffactorSubsystem);
      
    }
    
      @Override
    public void initialize(){
        algaeEndeffactorSubsystem.setSpeed(-12);
        new WaitCommand(pulse);
        algaeEndeffactorSubsystem.setSpeed(0);
    }

    @Override
    public void execute(){
       
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
