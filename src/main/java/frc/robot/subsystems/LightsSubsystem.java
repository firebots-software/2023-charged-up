package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Lights;

public class LightsSubsystem extends SubsystemBase {
    private int counter;
    private int lightsCount;
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private static LightsSubsystem front, back;

    private LightsSubsystem(int pwm) {
        counter = 0;
        lightsCount = Lights.LIGHT_SETTINGS_COUNT;
        m_led = new AddressableLED(pwm);
        m_ledBuffer = new AddressableLEDBuffer(Lights.LED_BUFFER_LENGTH);
        m_led.setLength(Lights.LED_BUFFER_LENGTH);

        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 35-i, 180, 0); // g r b
        }

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public static LightsSubsystem getFrontLed() {
        if (front == null) {
            front = new LightsSubsystem(Lights.LED_FRONT_PWM_PORT);
        }
        return front;
    }

    public static LightsSubsystem getBackLed() {
        if (back == null) {
        //    back = new LightsSubsystem(Lights.LED_BACK_PWM_PORT);
        }
        return front;
    }

    public void incrementer() {
        if (counter == lightsCount - 1) {
            counter = 0;
        } else
            counter++;
    }

    public void lightSetting0() {
        // ALL YELLOW - SIGNALS FOR CONE
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i,
                    Constants.Lights.YELLOW_3501[0],
                    Constants.Lights.YELLOW_3501[1],
                    Constants.Lights.YELLOW_3501[2]);
        }
        m_led.setData(m_ledBuffer);
    }

    public void lightSetting1() {
        // ALL BLUE - SIGNALS FOR CUBE
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i,
                    Constants.Lights.BLUE_3501[0],
                    Constants.Lights.BLUE_3501[1],
                    Constants.Lights.BLUE_3501[2]);
        }
        m_led.setData(m_ledBuffer);
    }

    private void lightSetting2(int firstRed) {
        // GRADIENT RED-YELLOW
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, 170, ((i + firstRed) % Lights.LED_BUFFER_LENGTH) * 3, 50);
        }

        m_led.setData(m_ledBuffer);
    }


    private void rainbow(int t) {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (t + (i * 180 / Lights.LED_BUFFER_LENGTH)) % 180;
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }

        m_led.setData(m_ledBuffer);
    }

    private void lightSettingAnimation(int t) {
        int thing;
        for (int i = 0; i < Lights.LED_BUFFER_LENGTH; i++) {
            thing = (i+t)%Lights.LED_BUFFER_LENGTH;
            m_ledBuffer.setRGB(i, (int)(0.6*t), 0, (int)(255-0.00004*t*t));
        }
        m_led.setData(m_ledBuffer);
    }

    private int t = 0;

    @Override
    public void periodic() {
        rainbow(t);
        t++;
    }
}
