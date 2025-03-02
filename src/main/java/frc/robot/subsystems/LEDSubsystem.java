package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
      private final AddressableLED m_led = new AddressableLED(9);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(210);
    private final LEDPattern m_rainBow = LEDPattern.rainbow(255, 120);
   public LEDSubsystem(){
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);

    m_led.start();

    setDefaultCommand(runPattern(m_rainBow));
   }
   @Override 
   public void periodic(){
    m_led.setData(m_ledBuffer);
   }

   public Command runPattern(LEDPattern pattern){
    return run(() -> pattern.applyTo(m_ledBuffer));
   }
    
}
