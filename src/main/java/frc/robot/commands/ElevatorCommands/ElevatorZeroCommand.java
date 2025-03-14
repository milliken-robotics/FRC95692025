package frc.robot.commands.ElevatorCommands;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorZeroCommand extends Command{
    private final ElevatorSubsystem elevatorSubsystem; 

    public ElevatorZeroCommand(ElevatorSubsystem elevatorSubsystem){
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);

    }
    
      @Override
    public void initialize(){
        elevatorSubsystem.setLevel(1);
    }

    @Override
    public void execute(){
        elevatorSubsystem.setPoint(30);
       
    }
    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.stop();
    }
    @Override
    public boolean isFinished(){
        return false;//Math.abs(900 - elevatorSubsystem.getPoint()) < 20; 
    }

}
