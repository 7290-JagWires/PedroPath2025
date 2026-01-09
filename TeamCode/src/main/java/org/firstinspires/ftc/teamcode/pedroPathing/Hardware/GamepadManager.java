package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;

/**
 * A helper class to manage gamepad button states and provide rising-edge detection.
 * This class turns a continuous button press into a single "just pressed" event.
 *
 * HOW TO USE:
 * 1. Create an instance in your OpMode: `private GamepadManager gamepadManager = new GamepadManager();`
 * 2. In your loop(), call update at the BEGINNING: `gamepadManager.update(gamepad1, gamepad2);`
 * 3. Check for button presses using the methods: `if (gamepadManager.a_just_pressed) { ... }`
 */
public class GamepadManager {

    // --- BUTTONS ---
    public boolean a_just_pressed;
    public boolean b_just_pressed;
    public boolean x_just_pressed;
    public boolean y_just_pressed;
    public boolean dpad_up_just_pressed;
    public boolean dpad_down_just_pressed;
    public boolean dpad_left_just_pressed;
    public boolean dpad_right_just_pressed;
    public boolean left_bumper_just_pressed;
    public boolean right_bumper_just_pressed;

    // --- TRIGGERS (as buttons) ---
    public boolean left_trigger_just_pressed;
    public boolean right_trigger_just_pressed;
    private static final double TRIGGER_THRESHOLD = 0.7; // Threshold to consider a trigger "pressed"

    // Internal state tracking maps
    private final Map<String, Boolean> previousState = new HashMap<>();
    private final Map<String, Boolean> currentState = new HashMap<>();

    public GamepadManager() {
        // Initialize all previous states to false
        previousState.put("a", false);
        previousState.put("b", false);
        previousState.put("x", false);
        previousState.put("y", false);
        previousState.put("dpad_up", false);
        previousState.put("dpad_down", false);
        previousState.put("dpad_left", false);
        previousState.put("dpad_right", false);
        previousState.put("left_bumper", false);
        previousState.put("right_bumper", false);
        previousState.put("left_trigger", false);
        previousState.put("right_trigger", false);
    }

    /**
     * Updates all button states. Call this ONCE at the beginning of your OpMode's loop().
     * @param gamepad The gamepad to read from (e.g., gamepad1 or gamepad2).
     */
    public void update(Gamepad gamepad) {
        // Read the current state of all buttons
        currentState.put("a", gamepad.a);
        currentState.put("b", gamepad.b);
        currentState.put("x", gamepad.x);
        currentState.put("y", gamepad.y);
        currentState.put("dpad_up", gamepad.dpad_up);
        currentState.put("dpad_down", gamepad.dpad_down);
        currentState.put("dpad_left", gamepad.dpad_left);
        currentState.put("dpad_right", gamepad.dpad_right);
        currentState.put("left_bumper", gamepad.left_bumper);
        currentState.put("right_bumper", gamepad.right_bumper);
        currentState.put("left_trigger", gamepad.left_trigger > TRIGGER_THRESHOLD);
        currentState.put("right_trigger", gamepad.right_trigger > TRIGGER_THRESHOLD);

        // Calculate the "just_pressed" event for each button
        a_just_pressed = currentState.get("a") && !previousState.get("a");
        b_just_pressed = currentState.get("b") && !previousState.get("b");
        x_just_pressed = currentState.get("x") && !previousState.get("x");
        y_just_pressed = currentState.get("y") && !previousState.get("y");
        dpad_up_just_pressed = currentState.get("dpad_up") && !previousState.get("dpad_up");
        dpad_down_just_pressed = currentState.get("dpad_down") && !previousState.get("dpad_down");
        dpad_left_just_pressed = currentState.get("dpad_left") && !previousState.get("dpad_left");
        dpad_right_just_pressed = currentState.get("dpad_right") && !previousState.get("dpad_right");
        left_bumper_just_pressed = currentState.get("left_bumper") && !previousState.get("left_bumper");
        right_bumper_just_pressed = currentState.get("right_bumper") && !previousState.get("right_bumper");
        left_trigger_just_pressed = currentState.get("left_trigger") && !previousState.get("left_trigger");
        right_trigger_just_pressed = currentState.get("right_trigger") && !previousState.get("right_trigger");


        // Update all previous states for the next loop iteration
        previousState.putAll(currentState);
    }
}
