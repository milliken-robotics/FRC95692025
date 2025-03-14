package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.util.*;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.*;

public class LEDSubsystem extends SubsystemBase {
      private final AddressableLED m_led = new AddressableLED(9);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(210);
    private LEDPattern pattern;
   
    public LEDSubsystem(){
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);

    m_led.start();
    // pattern = LEDPattern.rainbow(255, 120).scrollAtAbsoluteSpeed(
    //   MetersPerSecond.of(1), Meters.of(1 / 120.0));
      runDefaultColor();
    //setBlink(Color.kGreen);
    //setDefaultCommand(runPattern(pattern));
   }

   @Override 
   public void periodic(){
    pattern.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);


   }

   public void runDefaultColor(){
    var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
          if(alliance.get() == DriverStation.Alliance.Red){
            setContinousGradientScrolling(Color.kCoral, Color.kDarkRed, 0.8);
          }
          else{
            setContinousGradientScrolling(Color.kCyan, Color.kDarkBlue, 0.8);
          }
        }
        else{
          setRainbowScrolling();
        }
   }
  //   public Command runPattern(LEDPattern pattern){
  //   return run(() -> pattern.applyTo(m_ledBuffer));
  //  }

   public void setRainbow() {
     // For example, create a rainbow pattern with desired brightness and speed
     // parameters.
     pattern = LEDPattern.rainbow(255, 120);
   }
   public void setRainbowScrolling(){

      pattern = LEDPattern.rainbow(255, 120).scrollAtAbsoluteSpeed(
          MetersPerSecond.of(0.8), Meters.of(1 / 120.0));
   }

   // Sets all LEDs to a solid color.
   public void setSolidColor(Color color) {
     // Define a lambda pattern that fills the buffer with the provided color.
    pattern = LEDPattern.solid(color);
   }

   public void setBlink(Color color, double time){
    pattern = LEDPattern.solid(color).blink(Second.of(time));
   }
   public void setBreathe(Color color, double time){
    pattern = LEDPattern.solid(color).breathe(Second.of(time));
   }
   public void progressMask(Color color){
    pattern = LEDPattern.solid(color).breathe(Second.of(1));
   }
   public void setContinousGradientScrolling(Color color1, Color color2, double time){
    pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color1, color2).scrollAtAbsoluteSpeed(
      MetersPerSecond.of(time), Meters.of(1 / 120.0));
   }

   // Turns the LEDs off.
   public void setOff() {
     pattern = LEDPattern.solid(Color.kBlack);
   }
    
    
}
