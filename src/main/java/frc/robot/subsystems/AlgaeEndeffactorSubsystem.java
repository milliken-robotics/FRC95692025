package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareMap;

public class AlgaeEndeffactorSubsystem extends SubsystemBase{
    private final SparkMax algaeMotor = new SparkMax(HardwareMap.IT_ALGAE, MotorType.kBrushless);
    private final SparkMax algaePivotMotor = new SparkMax(HardwareMap.IT_ALGAE_PIVOT, MotorType.kBrushless);
    private final double zero = 0.54;

    // private final RelativeEncoder algaeMotorEncoder = algaeMotor.getEncoder();
 //   private final RelativeEncoder algaePivotMotorEncoder = algaePivotMotor.getEncoder();
    private final DutyCycleEncoder algaeDutyCycleEncoder = new DutyCycleEncoder(7);

    private final PIDController pid = new PIDController(6, 0, 0);

    public AlgaeEndeffactorSubsystem (){
    }
    
    public void algaeSetPoint(double target){
        double cur = algaeDutyCycleEncoder.get() - zero; 

        double output = pid.calculate(cur, target);
        output = output*algaePivotMotor.getBusVoltage()*-1;
        
        SmartDashboard.putNumber("target algae", target);

        SmartDashboard.putNumber("cur algae angle", cur);
       
        SmartDashboard.putNumber("algae ouput", output);

        algaePivotMotor.setVoltage(output);
    }

    public void setVolt (double volt){
        algaePivotMotor.setVoltage(volt);
    }
    @Override
    public void periodic(){
        //SmartDashboard.putNumber("current algae", algaePivotMotorEncoder.getPosition());
        SmartDashboard.putNumber("dio 7 algae", algaeDutyCycleEncoder.get() - zero);

    }
    public void algaePivotStop (){
        algaePivotMotor.setVoltage(0);
    }

    public void setSpeed(double volt){
        algaeMotor.setVoltage(volt);
    }
}
