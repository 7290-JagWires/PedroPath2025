package org.firstinspires.ftc.teamcode.pedroPathing.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Controller class for a PWM-controlled RGB Indicator Light.
 * This uses a Servo object to generate PWM signals.
 */
public class IndicatorLight {

    private final Servo pwmServo;

    /**
     * Constructor — supply the hardware map name for the PWM output.
     *
     * @param hwMap     The FTC hardware map.
     * @param servoName The name of the servo port in the config.
     */
    public IndicatorLight(HardwareMap hwMap, String servoName) {
        pwmServo = hwMap.get(Servo.class, servoName);

        // Ensure initialization to center/neutral
       // pwmServo.setPosition(0.5);
    }

    /**
     * Sends a raw PWM position to the indicator.
     * Value is 0.0-1.0, mapping 500–2500 µs roughly.
     *
     * @param position A normalized value (0.0 to 1.0).
     */
    public void setPwmPosition(double position) {
        position = Math.max(0.0, Math.min(1.0, position));
        pwmServo.setPosition(position);
    }

    /**
     * Simple presets. These values may need calibration on your specific hardware.
     */

    public void setRed() {
        setPwmPosition(0.05); // ~550 µs -> interpreted as red
    }

    public void setGreen() {
        setPwmPosition(0.47); // ~1000 µs -> interpreted as green
    }

    public void setPurple() {
        setPwmPosition(0.722); // ~1000 µs -> interpreted as green
    }

    public void setBlue() {
        setPwmPosition(0.66); // ~1500 µs -> interpreted as blue
    }

    public void setWhite() {
        setPwmPosition(1.0); // ~2000+ µs -> white
    }

    public void setGold() {
        setPwmPosition(0.300); // ~2000+ µs -> white
    }

    public void off() {
        setPwmPosition(0.0); // minimal PWM → off/black
    }
}

