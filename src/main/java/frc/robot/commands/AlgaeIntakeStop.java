package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeEndeffactorSubsystem;

public class AlgaeIntakeStop extends Command {
    private final AlgaeEndeffactorSubsystem algaeSubsystem;


    public AlgaeIntakeStop(AlgaeEndeffactorSubsystem algaeSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
        //addRequirements(algaeSubsystem);
    }

    @Override
    public void initialize() {

        algaeSubsystem.setSpeed(0);
        // timer.reset();
        // timer.start();
    }

    @Override
    public void execute() {
        // double currentTime = timer.get();
        // double cycleTime = currentTime % pulsePeriod;
        // // For most of the cycle, run at -12 volts.
        // if (cycleTime < (pulsePeriod - offDuration)) {
        // } else {
        //     algaeSubsystem.setSpeed(0);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.setSpeed(0);
        // timer.stop();
    }

    @Override
    public boolean isFinished() {
        // This command runs while the button is held.
        return false;
    }
}