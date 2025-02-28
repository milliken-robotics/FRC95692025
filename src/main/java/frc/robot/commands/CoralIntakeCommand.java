package frc.robot.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralEndeffactorSubsystem;

public class CoralIntakeCommand extends Command{
      private final CoralEndeffactorSubsystem coralEndeffactorSubsystem; 
      private final Timer time = new Timer();

    public CoralIntakeCommand(CoralEndeffactorSubsystem coralEndeffactorSubsystem){
        this.coralEndeffactorSubsystem = coralEndeffactorSubsystem;
        addRequirements(coralEndeffactorSubsystem);

    }
    
      @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        coralEndeffactorSubsystem.setVolt(12);
       
    }
    @Override
    public void end(boolean interrupted){
        new WaitCommand(0.03);
        coralEndeffactorSubsystem.stop();
    }
    @Override
    public boolean isFinished(){
        return !coralEndeffactorSubsystem.beamBroken();// Math.abs(900 - elevatorSubsystem.getPoint()) < 20; 
    }
}
