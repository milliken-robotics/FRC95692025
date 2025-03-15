package frc.robot.commands.CoralCommands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralEndeffactorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class CoralIntakeCommand extends Command{
      private final CoralEndeffactorSubsystem coralEndeffactorSubsystem; 
      private final Timer time = new Timer();
   private final LEDSubsystem ledSubsystem;


    public CoralIntakeCommand(CoralEndeffactorSubsystem coralEndeffactorSubsystem, LEDSubsystem ledSubsystem){
        this.coralEndeffactorSubsystem = coralEndeffactorSubsystem;
        this.ledSubsystem = ledSubsystem;
        addRequirements(coralEndeffactorSubsystem);
  

    }
    
      @Override
    public void initialize(){
        ledSubsystem.setSolidColor(Color.kYellow);
    }

    @Override
    public void execute(){
        if(!coralEndeffactorSubsystem.beamBroken1()){
            coralEndeffactorSubsystem.setVolt(4);
        }
        else{
            coralEndeffactorSubsystem.setVolt(12);
        }
       
    }
    @Override
    public void end(boolean interrupted){
        // new WaitCommand(0.045);
        coralEndeffactorSubsystem.stop();
        ledSubsystem.setSolidColor(Color.kGreen);
    }
    @Override
    public boolean isFinished(){
        return coralEndeffactorSubsystem.isObjectIn();// Math.abs(900 - elevatorSubsystem.getPoint()) < 20; 
    } // if not broken and broken
    //true not broken, false broken
}
