package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LIGHTS;

public class LightsSubsystem extends SubsystemBase {
    private static int counter;
    private static int lightsCount;
    private static AddressableLED m_led;
    private static AddressableLEDBuffer m_ledBuffer;
    private static LightsSubsystem instance;

    private LightsSubsystem() {
        counter = 0;
        lightsCount = LIGHTS.LIGHT_SETTINGS_COUNT;
        m_led = new AddressableLED(LIGHTS.LED_PWM_PORT);
        m_ledBuffer = new AddressableLEDBuffer(LIGHTS.LED_BUFFER_LENGTH);
        m_led.setLength(LIGHTS.LED_BUFFER_LENGTH);

        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 180, 0, 0);
        }

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public static LightsSubsystem getInstance(){
        if (instance == null) {
            instance = new LightsSubsystem();
          }
          return instance;
        }    
    

    public void incrementer() {
        if (counter == lightsCount-1) {
            counter = 0;
        } else
            counter++;
    }

    public void lightSetting0() {
        // ALL YELLOW - SIGNALS FOR CONE
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i,
                    Constants.LIGHTS.YELLOW_3501[0],
                    Constants.LIGHTS.YELLOW_3501[1],
                    Constants.LIGHTS.YELLOW_3501[2]);
        }
        m_led.setData(m_ledBuffer);
    }

    public void lightSetting1() {
        // ALL BLUE - SIGNALS FOR CUBE
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i,
                    Constants.LIGHTS.BLUE_3501[0],
                    Constants.LIGHTS.BLUE_3501[1],
                    Constants.LIGHTS.BLUE_3501[2]);
        }
        m_led.setData(m_ledBuffer);
    }

    private void lightSetting2(int firstRed) {
        // GRADIENT RED-YELLOW
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, 170, ((i+firstRed)%LIGHTS.LED_BUFFER_LENGTH) * 3, 50);
        }
    }

    private int t = 0;

    @Override
    public void periodic () {
        t = (t+1) % LIGHTS.LED_BUFFER_LENGTH;
        lightSetting2(t);
    }
}
