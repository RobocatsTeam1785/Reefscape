package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_buffer;
    private LEDPattern blue;
    private LEDPattern red;
    public Led(){
        m_led = new AddressableLED(0);


        blue = LEDPattern.solid(Color.kBlue);
        red =  LEDPattern.solid(Color.kBlue);
        // 60 cause it is default
        m_buffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_buffer.getLength());

        m_led.setData(m_buffer);
        m_led.start();
    }

    public void solidBlue(AddressableLED led){
        blue.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }
    public void SolidRed(AddressableLED led){
         
        red.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void initialization(){
        LEDPattern pattern = LEDPattern.progressMaskLayer(() -> m_buffer.getLength());

        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                blue.applyTo(m_buffer);
            }
            if (ally.get() == Alliance.Blue) {
                red.applyTo(m_buffer);
            }
        m_led.setData(m_buffer);
        }
        pattern.applyTo(m_buffer);
        
    }
   
}
    

