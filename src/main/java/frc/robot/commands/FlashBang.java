package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class FlashBang extends Command{

    private final LEDSubsystem ledSubsystem; 
    public FlashBang(LEDSubsystem ledSubsystem){
        this.ledSubsystem = ledSubsystem;

    }
    
      @Override
    public void initialize(){
      ledSubsystem.setBlink(Color.kWhite, 0.05);
    }

    // @Override
    // public void execute(){
    //     elevatorSubsystem.setPoint(ElevatorConstnats.L2);
       
    // }
    @Override
    public void end(boolean interrupted){
    //    ledSubsystem.setSolidColor(Color.kWhite);
    ledSubsystem.setRainbowScrolling();
    }
    @Override
    public boolean isFinished(){
        return false;// Math.abs(900 - elevatorSubsystem.getPoint()) < 20; 
    }
    
}

