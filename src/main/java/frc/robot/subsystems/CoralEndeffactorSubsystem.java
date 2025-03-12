package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareMap;

public class CoralEndeffactorSubsystem extends SubsystemBase{
    private final SparkMax leftCoralMotor = new SparkMax(HardwareMap.IT_CORAL_L, MotorType.kBrushless);
    private final SparkMax rightCoralMotor = new SparkMax(HardwareMap.IT_CORAL_R, MotorType.kBrushless);

     private final RelativeEncoder leftMotorEncoder = leftCoralMotor.getEncoder();
     private final RelativeEncoder rightMotorEncoder = rightCoralMotor.getEncoder(); 

     private double leftDiffRatio = 1; 
     private double rightDiffRatio = 0.4;

     private final DigitalInput beamBreak1 = new DigitalInput(HardwareMap.IT_BEAMBREAK1);
     private final DigitalInput beamBreak2 = new DigitalInput(HardwareMap.IT_BEAMBREAK2);

    public CoralEndeffactorSubsystem (){
        leftCoralMotor.setInverted(false);
        rightCoralMotor.setInverted(true);
    }

    public void setVolt (double volt){
        leftCoralMotor.setVoltage(volt);
        rightCoralMotor.setVoltage(volt);
    }
    public void setVoltDiff (double volt){
        leftCoralMotor.setVoltage(leftDiffRatio*volt);
        rightCoralMotor.setVoltage(rightDiffRatio*volt);
    }
    public void stop (){
        leftCoralMotor.setVoltage(0);
        rightCoralMotor.setVoltage(0);
    }

    public double getTurn (){
        return leftMotorEncoder.getPosition();
    }

    public void diffRight(){
        leftDiffRatio = 0.4;
        rightDiffRatio = 1; 
    }

    public void diffLeft(){
        leftDiffRatio = 1;
        rightDiffRatio = 0.4; 
    }

    public boolean beamBroken2(){
        return beamBreak2.get(); //(true not broken) false broke
    }

    public boolean beamBroken1() {
        return beamBreak1.get();
    }

    public boolean isObjectIn() {
        return beamBroken2(); //&& !this.beamBroken1(); 
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("BeamBroken2", beamBreak2.get());
        SmartDashboard.putBoolean("BeamBroken1", beamBreak1.get());
    }

}
