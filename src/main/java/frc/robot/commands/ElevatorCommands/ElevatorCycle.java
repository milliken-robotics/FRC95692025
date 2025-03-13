package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCycle extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double dir;

    public ElevatorCycle(ElevatorSubsystem elevatorSubsystem, double dir) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.dir = dir;
        addRequirements(elevatorSubsystem);

    }

    @Override
    public void initialize() {

        int curlevel = elevatorSubsystem.getLevel();
        curlevel += dir;
        Command run; 
        
        if (curlevel == 3) {
            run = new ElevatorL3Command(elevatorSubsystem);
        }
       else if (curlevel == 2) {
           run = new ElevatorL2Command(elevatorSubsystem);

        }
        else if (curlevel == 1) {
            run = new ElevatorZeroCommand(elevatorSubsystem);
        }
        else if (curlevel == 0) {
            run = new ElevatorL3Command(elevatorSubsystem);
        }
        else{
            run = new ElevatorZeroCommand(elevatorSubsystem);
        }
        run.schedule();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;// Math.abs(900 - elevatorSubsystem.getPoint()) < 20;
    }

}
