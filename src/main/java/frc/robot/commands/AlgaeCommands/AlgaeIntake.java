package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeEndeffactorSubsystem;

public class AlgaeIntake extends Command {
    private final AlgaeEndeffactorSubsystem algaeSubsystem;
    public AlgaeIntake(AlgaeEndeffactorSubsystem algaeSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
        //addRequirements(algaeSubsystem);
    }

    @Override
    public void initialize() {
        
        algaeSubsystem.setSpeed(-12);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}