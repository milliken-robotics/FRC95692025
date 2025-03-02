package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareMap;

public class AlgaeEndeffactorSubsystem extends SubsystemBase{
    private final SparkMax algaeMotor = new SparkMax(HardwareMap.IT_ALGAE, MotorType.kBrushless);
    private final SparkMax algaePivotMotor = new SparkMax(HardwareMap.IT_ALGAE_PIVOT, MotorType.kBrushless);

    // private final RelativeEncoder algaeMotorEncoder = algaeMotor.getEncoder();
    private final RelativeEncoder algaePivotMotorEncoder = algaePivotMotor.getEncoder();

    private final PIDController pid = new PIDController(1, 0, 0);

    public AlgaeEndeffactorSubsystem (){
    }
    
    public void algaeSetPoint(double target){
        double output = pid.calculate(algaePivotMotorEncoder.getPosition(), target);
        output = output*algaePivotMotor.getBusVoltage();
        SmartDashboard.putNumber("target algae", target);
       
        SmartDashboard.putNumber("algae ouput", output);

        algaePivotMotor.setVoltage(output);
    }

    public void setVolt (double volt){
        algaePivotMotor.setVoltage(volt);
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("current algae", algaePivotMotorEncoder.getPosition());

    }
    public void algaePivotStop (){
        algaePivotMotor.setVoltage(0);
    }

    public void setSpeed(double volt){
        algaeMotor.setVoltage(volt);
    }
}
