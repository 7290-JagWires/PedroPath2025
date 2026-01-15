package org.firstinspires.ftc.teamcode.TeleOp.Sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * MagneticLimitSwitch - simple Hall effect sensor helper.
 *
 * - isTriggered() → TRUE when magnet is present
 * - wasJustTriggered() → TRUE only at the moment the magnet first appears
 * - update() → call once per loop
 */
public class TeleOpMagneticLimitSwitch {

    private final DigitalChannel hall;

    private boolean lastRaw = false;  // last loop state
    private boolean justTriggered = false;  // rising edge flag

    public TeleOpMagneticLimitSwitch(HardwareMap hardwareMap, String deviceName) {
        hall = hardwareMap.get(DigitalChannel.class, deviceName);
        hall.setMode(DigitalChannel.Mode.INPUT);
    }

    /** Raw sensor reading (Hall sensors are ACTIVE LOW) */
    public boolean isTriggered() {
        return !hall.getState();   // LOW = triggered
    }

    /**
     * Returns TRUE only for one loop cycle when the magnet first appears.
     * (We use this for drift correction.)
     */
    public boolean wasJustTriggered() {
        return justTriggered;
    }

    /**
     * Call this ONCE PER LOOP.
     * Handles "rising edge" logic.
     */
    public void update() {
        boolean raw = isTriggered();

        // Rising edge detection: raw is true NOW, but was false LAST loop
        justTriggered = raw && !lastRaw;

        // Store last state
        lastRaw = raw;
    }
}
