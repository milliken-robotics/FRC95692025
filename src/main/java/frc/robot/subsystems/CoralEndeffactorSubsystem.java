package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareMap;

public class CoralEndeffactorSubsystem extends SubsystemBase{
    private final SparkMax leftCoralMotor = new SparkMax(HardwareMap.IT_CORAL_L, MotorType.kBrushless);
    private final SparkMax rightCoralMotor = new SparkMax(HardwareMap.IT_CORAL_R, MotorType.kBrushless);
    private final DigitalInput beamBreak = new DigitalInput(HardwareMap.IT_BEAMBREAK); 

    public CoralEndeffactorSubsystem (){
        leftCoralMotor.setInverted(false);
        rightCoralMotor.setInverted(true);
    }

    public void setVolt (double volt){
        leftCoralMotor.setVoltage(volt);
        rightCoralMotor.setVoltage(volt);
    }
    public void stop (){
        leftCoralMotor.setVoltage(0);
        rightCoralMotor.setVoltage(0);
    }

    public boolean beamBroken (){
        return beamBreak.get(); //(true not broken) false broke
    }

    @Override
    public void periodic(){

        SmartDashboard.putBoolean("BeamBroken", beamBreak.get());
    }

}
