package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareMap;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax elevatorMotorLeft = new SparkMax(HardwareMap.IT_ELEVATOR_L, MotorType.kBrushless);
    private final SparkMax elevatorMotorRight = new SparkMax(HardwareMap.IT_ELEVATOR_R, MotorType.kBrushless);

    private final RelativeEncoder leftMotorEncoder = elevatorMotorLeft.getEncoder();
    private final RelativeEncoder rightMotorEncoder = elevatorMotorRight.getEncoder();

    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.2, 0.24, 8.97, 0.02);

    private final PIDController pid = new PIDController(0.05, 0, 0);

    private double  currentLeft, currentRight = 0 ; 

    public int level = 1; 

    public ElevatorSubsystem(){

        elevatorMotorLeft.setInverted(true);

        leftMotorEncoder.setPosition(0);
        rightMotorEncoder.setPosition(0);
       // elevatorMotorLeft.configure(null, null, null)
    }

    public void setLevel(int curlevel){
        level = curlevel;
    }
    public int getLevel(){
        return level; 
    }


    int doublecheck = 0 ; 
    @Override
    public void periodic(){
        doublecheck ++; 
        // currentLeft = leftMotorEncoder.getPosition() - initLeft; 
        // currentRight = rightMotorEncoder.getPosition() - initRight; 
        currentLeft = leftMotorEncoder.getPosition(); 
        currentRight = rightMotorEncoder.getPosition(); 


        // SmartDashboard.putNumber("current left", currentLeft);
        // SmartDashboard.putNumber("current right", currentRight);

        // SmartDashboard.putNumber("double check", doublecheck); 
        SmartDashboard.putNumber("cur level", level);
        
    }
    public void setPoint (double target){
        double pidOutputLeft = pid.calculate(currentLeft, target);
        double pidOutputRight = pid.calculate(currentRight, target);

        double feedforwardOutput = feedforward.calculate(2,3); 

        double outLeft = pidOutputLeft;//+feedforwardOutput; 
        double outRight = pidOutputRight;// + feedforwardOutput;
        
        // SmartDashboard.putNumber("PID/setpoint", target);
        // SmartDashboard.putNumber("PID/Measurment", getPoint());
        // SmartDashboard.putNumber("PID/Output", outLeft); 

        // SmartDashboard.putNumber("feedforward", feedforwardOutput);
        // SmartDashboard.putNumber("error points", pid.getError());
        outLeft = outLeft*elevatorMotorLeft.getBusVoltage(); 
        outRight = outRight*elevatorMotorRight.getBusVoltage(); 

        // outLeft = MathUtil.clamp(outLeft, -6,6);
        // outRight = MathUtil.clamp(outRight, -6,6);

        // SmartDashboard.putNumber("outVolt left", outLeft);
        // SmartDashboard.putNumber("OutVolt right", outRight);

        elevatorMotorLeft.setVoltage(pidOutputLeft);
        elevatorMotorRight.setVoltage(pidOutputRight);
    }

    public void setPointInternal (){

    }

    public double getPoint(){

        return (currentLeft+currentRight)/2;

    }

    public void setVolt(double volt){
        elevatorMotorLeft.setVoltage(volt);
        elevatorMotorRight.setVoltage(volt);
    }
    public void stop(){
        elevatorMotorLeft.set(0);
        elevatorMotorRight.set(0);
    }
    
}
