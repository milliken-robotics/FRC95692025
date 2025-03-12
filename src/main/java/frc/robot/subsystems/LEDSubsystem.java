package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.util.*;
import edu.wpi.first.units.Units.*;

public class LEDSubsystem extends SubsystemBase {
      private final AddressableLED m_led = new AddressableLED(9);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(210);
    private LEDPattern pattern;
   
    public LEDSubsystem(){
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);

    m_led.start();
    pattern = LEDPattern.rainbow(255, 120);
    setDefaultCommand(runPattern(pattern));
   }

   @Override 
   public void periodic(){
    m_led.setData(m_ledBuffer);
   }

    public Command runPattern(LEDPattern pattern){
    return run(() -> pattern.applyTo(m_ledBuffer));
   }

   public void setRainbow() {
     // For example, create a rainbow pattern with desired brightness and speed
     // parameters.
     pattern = LEDPattern.rainbow(255, 120);
   }

   // Sets all LEDs to a solid color.
   public void setSolidColor(Color color) {
     // Define a lambda pattern that fills the buffer with the provided color.
    pattern = LEDPattern.solid(color);
   }

   public void setBlink(){
    //pattern = pattern.blink(0.5);
   }

   // Turns the LEDs off.
   public void setOff() {
     pattern = LEDPattern.solid(Color.kBlack);
   }
    
    
}
