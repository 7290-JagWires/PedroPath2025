package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.IndicatorLight;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Door;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.IntakeActive;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Logic.SpindexerIndexerLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Door;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Spindexer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Robot;


import com.qualcomm.robotcore.util.ElapsedTime;


public class Utilities {
    /**
     * Checks if the color sensor detects a specific "ball" color (purple or green) and updates an indicator light accordingly.
     * This method first updates the color sensor to get the latest reading. It then checks if the detected
     * color is either purple or green. If it is purple, the indicator light is set to purple. If it is
     * green, the indicator light is set to green. In both cases, the method returns true. If neither
     * color is detected, the indicator light is turned off and the method returns false.
     *
     * @param colorSensor The {@link ColorSensor} instance to read the color from.
     * @param indicator The {@link IndicatorLight} instance to update based on the detected color.
     * @return Returns {@code true} if a purple or green ball is detected, {@code false} otherwise.
     */
    public static boolean isBall(ColorSensor colorSensor, IndicatorLight indicator) {
        colorSensor.update();
        ColorSensor.BallColor color = colorSensor.getBallColor();

        if (color == ColorSensor.BallColor.PURPLE) {
            indicator.setPurple();
            return true;
        }
        else if (color == ColorSensor.BallColor.GREEN) {
            indicator.setGreen();
            return true;
        }
        else  {
            indicator.off();
            return false;
        }
    }

    /**
     * Handles standard robot-centric mecanum drive controls.
     * Reads joystick values from the gamepad and sends power commands to the robot's drive system.
     *
     * @param gamepad The gamepad to read joystick inputs from (typically gamepad1).
     * @param robot   The main Robot object that contains the mecanumDrive system.
     */
// Inside a new or existing Utilities.java file

    public static void handleRobotCentricDrive(Gamepad gamepad, Robot robot) {
        double y = gamepad.left_stick_y;
        double x = -gamepad.left_stick_x;
        double r = -gamepad.right_stick_x;

        // Create a deadzone to prevent joystick drift
        double deadzone = 0.05; // 5% deadzone, adjust as needed

        if (Math.abs(y) < deadzone) y = 0;
        if (Math.abs(x) < deadzone) x = 0;
        if (Math.abs(r) < deadzone) r = 0;

        // Only apply power if there is significant input
        if (y != 0 || x != 0 || r != 0) {
            robot.mecanumDrive.drive(x, y, r);
        } else {
            robot.mecanumDrive.stop();
        }
    }
    public static class PickupManager {
        // Enums for state tracking
        public enum PickupState { IDLE, SPIN_UP, PICKUP, WAITING_FOR_SPIN, FINISHED }

        // State variables
        private PickupState pickupState = PickupState.IDLE;
        private int pickupCount = 0;
        private int totalBallCount = 0;

        // Hardware dependencies
        private final Door door;
        private final Shooter shooter;
        private final IntakeActive intakeActive;
        private final SpindexerIndexerLogic spindexerLogic;
        private final ColorSensor colorSensor;
        private final IndicatorLight indicator;

        // Timers
        private final ElapsedTime pickupTimer = new ElapsedTime();

        private static final int LAUNCH_DELAY_MS = 500; // Example delay

        /**
         * Initializes the PickupManager with all the necessary hardware components and logic modules.
         * This constructor takes instances of hardware controllers (Door, Shooter, IntakeActive),
         * sensor wrappers (ColorSensor, IndicatorLight), and logic handlers (SpindexerIndexerLogic)
         * that the manager will orchestrate to perform the pickup sequence.
         *
         * @param door The {@link Door} object to control the intake door.
         * @param shooter The {@link Shooter} object, used here to ensure it's stopped during pickup.
         * @param intakeActive The {@link IntakeActive} object to control the intake mechanism.
         * @param spindexerLogic The {@link SpindexerIndexerLogic} to manage the spindexer and indexer.
         * @param colorSensor The {@link ColorSensor} to detect incoming balls.
         * @param indicator The {@link IndicatorLight} to provide visual feedback on ball detection.
         */
        public PickupManager(Door door, Shooter shooter, IntakeActive intakeActive, SpindexerIndexerLogic spindexerLogic, ColorSensor colorSensor, IndicatorLight indicator) {
            this.door = door;
            this.shooter = shooter;
            this.intakeActive = intakeActive;
            this.spindexerLogic = spindexerLogic;
            this.colorSensor = colorSensor;
            this.indicator = indicator;
        }

        /**
         * Initiates the pickup sequence.
         * This method sets the state to {@code SPIN_UP}, unlocks and closes the door,
         * resets the pickup counter, and stops the shooter. This prepares the robot to start
         * looking for and picking up items.
         */
        public void start() {
            pickupState = PickupState.SPIN_UP;
            door.unlock();
            door.forceClose();
            pickupCount = 0;
            shooter.stop();
        }

        /**
         * Updates the state machine for the pickup process. This method should be called repeatedly in a loop.
         * It manages the transitions between different states of the pickup cycle.
         * <p>
         * The state machine progresses as follows:
         * <ul>
         *   <li><b>IDLE:</b> The machine is inactive.</li>
         *   <li><b>SPIN_UP:</b> Prepares for pickup by closing the door, starting the intake, and then
         *       immediately transitioning to the PICKUP state.</li>
         *   <li><b>PICKUP:</b> After a brief delay (LAUNCH_DELAY_MS), it checks the color sensor for a ball.
         *       If a ball is detected, it increments the pickup and total ball counts. If the robot still needs
         *       to collect more balls (totalBallCount < 3), it commands the spindexer to move to the next
         *       compartment and enters the WAITING_FOR_SPIN state. Otherwise, it transitions to FINISHED.</li>
         *   <li><b>WAITING_FOR_SPIN:</b> Waits for the spindexer to finish rotating to the next compartment,
         *       confirmed by the limit switch. Once the spindexer is in position, it either returns to the
         *       PICKUP state to look for another ball or transitions to FINISHED if the required number of
         *       balls for this cycle has been picked up.</li>
         *   <li><b>FINISHED:</b> The pickup cycle is complete. The machine will remain in this state until
         *       {@link #start()} is called again.</li>
         * </ul>
         */
        public void update() {
            switch (pickupState) {
                case IDLE:
                    break;
                case SPIN_UP:
                    door.unlock();
                    door.forceClose();
                    intakeActive.intakeOn();
                    pickupTimer.reset();
                    pickupState = PickupState.PICKUP;
                    break;
                case PICKUP:
                    if (pickupTimer.milliseconds() > LAUNCH_DELAY_MS) {
                        if (Utilities.isBall(colorSensor, indicator)) {
                            pickupCount++;
                            totalBallCount++;
                            if (totalBallCount < 3) {
                                spindexerLogic.nextCompartment();
                                pickupState = PickupState.WAITING_FOR_SPIN;
                            } else {
                                totalBallCount = 0;
                                pickupState = PickupState.FINISHED;
                            }
                        }
                    }
                    break;
                case WAITING_FOR_SPIN:
                        if (!spindexerLogic.isBusy()) {
                            pickupTimer.reset();
                            if (pickupCount == 1) {
                                pickupState = PickupState.FINISHED;
                            } else {
                                pickupState = PickupState.PICKUP;
                            }
                    }
                    break;
                case FINISHED:
                    // You might want to turn the intake off here
                    // intakeActive.intakeOff();
                    break;
            }
        }

        /**
         * Retrieves the current state of the pickup state machine.
         *
         * @return The current {@link PickupState} enum value, representing the state machine's progress.
         */
        public PickupState getState() {
            return pickupState;
        }

        /**
         * Checks if the pickup sequence has completed.
         *
         * @return Returns {@code true} if the current state is {@link PickupState#FINISHED},
         *         indicating the pickup process is done. Returns {@code false} otherwise.
         */
        public boolean isFinished() {
            return pickupState == PickupState.FINISHED;
        }

        /**
         * Sets the total number of balls the robot is currently holding.
         * This can be used to manually override or initialize the ball count, for example,
         * at the beginning of a match or after a manual reset.
         *
         * @param count The new total ball count.
         */
        public void setTotalBallCount(int count) {
            this.totalBallCount = count;
        }

        public int getTotalBallCount() {
            return this.totalBallCount;
        }
    }

    /**
     * Manages the automated process of dumping/shooting balls from the robot.
     * <p>
     * This class implements a state machine to control the sequence of actions required to shoot
     * a predefined number of balls (typically 3). It coordinates the shooter, the spindexer, and
     * the door to ensure a reliable dumping process. The manager handles spinning up the shooter,
     * opening the door, and rotating the spindexer to feed each ball into the shooter mechanism.
     * <p>
     * The state machine progresses through the following states:
     * <ul>
     *   <li>{@link DumpState#IDLE}: The initial state where the manager is inactive.</li>
     *   <li>{@link DumpState#SPIN_UP}: The shooter motor is started and given time to reach its target speed.</li>
     *   <li>{@link DumpState#DUMPING}: The door is opened, and the spindexer is sequentially rotated
     *       to feed each ball into the shooter. This state repeats until all balls are dumped.</li>
     *   <li>{@link DumpState#FINISHED}: The dumping cycle is complete. The manager will remain in
     *       this state until it is started again.</li>
     * </ul>
     * To use this class, create an instance with the required hardware components and call the
     * {@link #update()} method repeatedly in the main loop of an OpMode. The {@link #start()}
     * method begins the dumping sequence.
     */
    public static class DumpManager {

        // Enum for state tracking
        public enum DumpState { IDLE, SPIN_UP, DUMPING, FINISHED }

        // State variables
        private DumpState dumpState = DumpState.IDLE;
        private int dumpCount = 0;
        private final ElapsedTime dumpTimer = new ElapsedTime();
        private final ElapsedTime launchTimer = new ElapsedTime();

        // Hardware dependencies
        private final Shooter shooter;
        private final Door door;
        private final SpindexerIndexerLogic spindexerLogic;
        private final SpindexerRotator spindexerRotator; // <-- ADD THIS LINE
        private final OpMode opMode;
        // Constants
        private static final int SHOOTER_SPINUP_MS = 2000;
        private static final int LAUNCH_DELAY_MS = 400;
        private static final int TOTAL_BALLS_TO_DUMP = 3;

        private boolean isTeleOpDumping = false;
        private boolean prevTriggerState = false;


        /**
         * Initializes the DumpManager with the necessary hardware components.
         * This constructor takes instances of hardware controllers (Shooter, Door) and logic handlers
         * (SpindexerIndexerLogic) that the manager will use to execute the dumping sequence.
         *
         * @param shooter The {@link Shooter} object responsible for launching the balls.
         * @param door The {@link Door} object used to control the opening/closing of the ball gate.
         * @param spindexerLogic The {@link SpindexerIndexerLogic} to manage rotating the spindexer to the next ball.
         */
        public DumpManager(OpMode opMode,Shooter shooter, Door door, SpindexerIndexerLogic spindexerLogic, SpindexerRotator spindexerRotator) {
            this.shooter = shooter;
            this.door = door;
            this.spindexerLogic = spindexerLogic;
            this.spindexerRotator = spindexerRotator;
            this.opMode = opMode;
        }


        /**
         * Initiates the pickup sequence.
         * This method sets the state to {@code SPIN_UP}, unlocks and closes the door,
         * resets the pickup counter, and stops the shooter. This prepares the robot to start
         * looking for and picking up items.
         */
        public void start() {
            if (dumpState == DumpState.IDLE || dumpState == DumpState.FINISHED) {
                this.dumpState = DumpState.SPIN_UP;
                this.dumpCount = 0;
                this.dumpTimer.reset();
                this.shooter.run();
            }
        }

        public void stop() {
            dumpState = DumpState.IDLE; // **THIS IS THE CRITICAL FIX**
        }


        /**
         * Updates the state machine for the pickup process. This method should be called repeatedly in a loop.
         * It manages the transitions between different states of the pickup cycle.
         * <p>
         * The state machine progresses as follows:
         * <ul>
         *   <li><b>IDLE:</b> The machine is inactive.</li>
         *   <li><b>SPIN_UP:</b> Prepares for pickup by closing the door, starting the intake, and then
         *       immediately transitioning to the PICKUP state.</li>
         *   <li><b>PICKUP:</b> After a brief delay ({@code LAUNCH_DELAY_MS}), it checks the color sensor for a ball.
         *       If a ball is detected, it increments the pickup and total ball counts. If the robot still needs
         *       to collect more balls (totalBallCount < 3), it commands the spindexer to move to the next
         *       compartment and enters the WAITING_FOR_SPIN state. Otherwise, it transitions to FINISHED.</li>
         *   <li><b>WAITING_FOR_SPIN:</b> Waits for the spindexer to finish rotating to the next compartment,
         *       confirmed by {@link SpindexerIndexerLogic#()}. Once the spindexer
         *       is in position, it either returns to the PICKUP state to look for another ball or transitions
         *       to FINISHED if the required number of balls for this cycle ({@code pickupCount}) has been collected.</li>
         *   <li><b>FINISHED:</b> The pickup cycle is complete. The machine will remain in this state until
         *       {@link #start()} is called again.</li>
         * </ul>
         */
        public void update() {
            // Always keep the rotator updated
            spindexerRotator.update();

            switch (dumpState) {
                case IDLE:
                    break;
                case SPIN_UP:
                    if (isTeleOpDumping) {
                        spindexerRotator.start(TOTAL_BALLS_TO_DUMP);
                        dumpState = DumpState.DUMPING;
                    } else if (dumpTimer.milliseconds() >= SHOOTER_SPINUP_MS) {
                        spindexerRotator.start(TOTAL_BALLS_TO_DUMP);
                        launchTimer.reset();
                        dumpState = DumpState.DUMPING;
                    }
                    break;
                case DUMPING:
                    door.forceOpenLock();
                    // Wait for the SpindexerRotator to confirm it has finished all 3 rotations
                    if (spindexerRotator.isFinished()) {
                        dumpState = DumpState.FINISHED;
                    }
                    break;
                case FINISHED:
                    // Actions to take once all balls are dumped
                    stop();
                    break;
            }
        }

        /**
         * Retrieves the current state of the pickup process.
         * This can be used to monitor the progress of the pickup state machine from outside the class.
         *
         * @return The current {@link DumpState} enum value (IDLE, SPIN_UP, PICKUP, WAITING_FOR_SPIN, or FINISHED).
         */
        public DumpState getState() {
            return dumpState;
        }

        /**
         * Checks if the pickup sequence has completed.
         *
         * @return Returns {@code true} if the current state is {@code FINISHED}, indicating that the
         * pickup cycle is complete. Returns {@code false} otherwise.
         */
        public boolean isFinished() {
            return dumpState == DumpState.FINISHED;
        }
        public void updateTeleOp(boolean triggerPressed) {
            // 1. Handle the toggle logic
            if (triggerPressed && !prevTriggerState) {
                isTeleOpDumping = !isTeleOpDumping;

                // If we are starting a new dump sequence...
                if (isTeleOpDumping) {
                    // Call the existing autonomous start() method to begin the sequence
                    this.start();
                }
            }
            prevTriggerState = triggerPressed;

            // 2. Run the state machine if we are in dump mode
            if (isTeleOpDumping) {
                // Open the door while dumping
                door.forceOpenLock();

                // The existing update() method runs the core state machine (SPIN_UP, DUMPING, etc.)
                this.update();

                // 3. Check if the sequence has finished
                if (this.isFinished()) {
                    isTeleOpDumping = false; // Turn off dump mode

                    // Reset hardware to a safe state
                    door.unlock();
                    door.forceClose();
                }
            }
        }

        /**
         * A getter to check if the TeleOp dump mode is active.
         * @return true if the dump sequence is running, false otherwise.
         */
        public boolean isDumping() {
            return isTeleOpDumping;
        }
                public int getDumpCount() {
            return dumpCount;
        }
    }


    /**
     * Manages a multi-step rotation of the spindexer.
     * <p>
     * This class implements a state machine to rotate the spindexer a specified number of times (1 or 2).
     * It is designed to handle timed pauses between rotations to ensure reliable hardware operation.
     * The state machine progresses as follows:
     * <ul>
     *   <li><b>IDLE:</b> The machine is inactive.</li>
     *   <li><b>START_ROTATION:</b> Initiates the first rotation of the spindexer and moves to a waiting state.</li>
     *   <li><b>WAIT_FOR_FIRST_SPIN:</b> Waits for the limit switch to confirm the first rotation is complete.
     *       If more rotations are needed, it pauses before starting the next one. Otherwise, it finishes.</li>
     *   <li><b>START_SECOND_ROTATION:</b> After a delay, it initiates the second rotation.</li>
     *   <li><b>WAIT_FOR_SECOND_SPIN:</b> Waits for the limit switch to confirm the second rotation is complete, then finishes.</li>
     *   <li><b>FINISHED:</b> The rotation sequence is complete.</li>
     * </ul>
     */
    public static class SpindexerRotator {

        public enum RotateState {IDLE, START_ROTATION, WAIT_FOR_FIRST_SPIN, START_SECOND_ROTATION, WAIT_FOR_SECOND_SPIN, FINISHED}

        private RotateState rotateState = RotateState.IDLE;
        private final SpindexerIndexerLogic spindexerLogic;
        private final ElapsedTime timer = new ElapsedTime();
        private int rotationsNeeded = 0;

        private static final int ROTATION_PAUSE_MS = 500; // Pause between spins

        /**
         * Initializes the SpindexerRotator with necessary hardware logic.
         *
         * @param spindexerLogic The SpindexerIndexerLogic to manage spindexer movement.
         */
        public SpindexerRotator(SpindexerIndexerLogic spindexerLogic) {
            this.spindexerLogic = spindexerLogic;
        }

        /**
         * Starts the rotation sequence.
         *
         * @param numberOfRotations The number of compartments to rotate (1 or 2).
         */
        public void start(int numberOfRotations) {
            if (rotateState == RotateState.IDLE || rotateState == RotateState.FINISHED) {
                if (numberOfRotations > 0 && numberOfRotations <= 2) {
                    this.rotationsNeeded = numberOfRotations;
                    this.rotateState = RotateState.START_ROTATION;
                } else {
                    // If 0 or invalid number, just go straight to finished.
                    this.rotationsNeeded = 0;
                    this.rotateState = RotateState.FINISHED;
                }
            }
        }

        /**
         * Updates the state machine for the rotation process. Call this method in a loop.
         */
        public void update() {
            switch (rotateState) {
                case IDLE:
                    break;

                case START_ROTATION:
                    spindexerLogic.nextCompartment(); // Start the first spin
                    timer.reset();
                    rotateState = RotateState.WAIT_FOR_FIRST_SPIN;
                    break;

                case WAIT_FOR_FIRST_SPIN:
                    // Wait for the hardware to confirm the spin is complete
                    if (!spindexerLogic.isBusy()) {
                        if (rotationsNeeded == 1) {
                            // If we only needed one spin, we are done.
                            rotateState = RotateState.FINISHED;
                        } else {
                            // If we need another spin, start the timer for a pause.
                            timer.reset();
                            rotateState = RotateState.START_SECOND_ROTATION;
                        }
                    }
                    break;

                case START_SECOND_ROTATION:
                    // Wait for the pause to complete before starting the next spin
                    if (timer.milliseconds() > ROTATION_PAUSE_MS) {
                        spindexerLogic.nextCompartment(); // Start the second spin
                        rotateState = RotateState.WAIT_FOR_SECOND_SPIN;
                    }
                    break;

                case WAIT_FOR_SECOND_SPIN:
                    // Wait for the hardware to confirm the second spin is complete
                    if (!spindexerLogic.isBusy()) {
                        rotateState = RotateState.FINISHED;
                    }
                    break;

                case FINISHED:
                    // Do nothing until started again.
                    break;
            }
        }

        /**
         * Checks if the rotation sequence has completed.
         *
         * @return Returns true if the state is FINISHED.
         */
        public boolean isFinished() {
            return rotateState == RotateState.FINISHED;
        }
    }

    public static class ManualShootManager {

        // 1. Move the enum for state tracking inside the manager
        public enum ShootState {
            IDLE,        // Waiting for input
            OPENING_DOOR,  // The door is opening
            CLOSING_DOOR   // The door is closing
        }
        private ShootState shootState = ShootState.IDLE;

        // 2. Move hardware dependencies here
        private final Door door;
        private final SpindexerIndexerLogic spindexerLogic;

        // 3. Move the timer here
        private final ElapsedTime doorTimer = new ElapsedTime();
        private static final int SERVO_MOVE_TIME_MS = 300;

        // 4. Constructor to get the required hardware components
        public ManualShootManager(Door door, SpindexerIndexerLogic spindexerLogic) {
            this.door = door;
            this.spindexerLogic = spindexerLogic;
        }

        /**
         * The main update loop for the manual shoot state machine.
         * Call this in every loop of your OpMode.
         * @param shootButtonPressed Whether the shoot button is currently pressed.
         * @param isDumpModeActive   Whether another action (like auto-dump) is happening.
         */
        public void update(boolean shootButtonPressed, boolean isDumpModeActive) {
            // Always update the spindexer's internal logic for its own state machine.
            spindexerLogic.update();

            switch (shootState) {
                case IDLE:
                    // If the button is pressed and we're not busy with another sequence...
                    if (shootButtonPressed && !isDumpModeActive) {
                        // 1. Open the door
                        door.forceOpenLock();
                        doorTimer.reset();
                        // 2. Move to the next state
                        shootState = ShootState.OPENING_DOOR;
                    }
                    break;

                case OPENING_DOOR:
                    // Wait for a short time for the servo to move
                    if (doorTimer.milliseconds() > SERVO_MOVE_TIME_MS) {
                        // 1. Close the door
                        door.forceClose();
                        doorTimer.reset();
                        // 2. Move to the next state
                        shootState = ShootState.CLOSING_DOOR;
                    }
                    break;

                case CLOSING_DOOR:
                    // Wait again for the servo to close
                    if (doorTimer.milliseconds() > SERVO_MOVE_TIME_MS) {
                        // 1. Advance the spindexer to the next compartment
                        spindexerLogic.nextCompartment();
                        // 2. Return to IDLE, ready for the next button press
                        shootState = ShootState.IDLE;
                    }
                    break;
            }
        }

        /**
         * Checks if the manager is currently busy with a shooting sequence.
         */
        public boolean isShooting() {
            return shootState != ShootState.IDLE;
        }

        public ShootState getState() {
            return shootState;
        }
    }
    /**
     * Manages a very simple manual pickup (indexing) action.
     * When commanded, this class will rotate the spindexer by one compartment and then stop.
     * This is useful for manual adjustments or loading a single ball without the full intake process.
     */
    public static class ManualPickupManager {

        // CORRECTED: A simpler two-state machine is more robust.
        public enum ManualPickupState { IDLE, WAIT_FOR_SPIN }
        private ManualPickupState state = ManualPickupState.IDLE;

        private final SpindexerIndexerLogic spindexerLogic;

        /**
         * Initializes the manager with the spindexer logic controller.
         * @param spindexerLogic The logic controller for the spindexer.
         */
        public ManualPickupManager(SpindexerIndexerLogic spindexerLogic) {
            this.spindexerLogic = spindexerLogic;
        }

        /**
         * The main update loop. Call this every cycle.
         * @param pickupButtonPressed A rising-edge signal (wasJustPressed) for the pickup button.
         */
        public void update(boolean pickupButtonPressed) {
            // Always update the underlying spindexer logic.
            spindexerLogic.update();

            switch (state) {
                case IDLE:
                    // If the button is pressed, command the spindexer to move and change state.
                    if (pickupButtonPressed) {
                        spindexerLogic.nextCompartment();
                        // Immediately move to the waiting state.
                        state = ManualPickupState.WAIT_FOR_SPIN;
                    }
                    break;
                // REMOVED the faulty INDEXING state.

                case WAIT_FOR_SPIN:
                    // Wait for the spindexer to report that its rotation is complete.
                    // The spindexerLogic is now guaranteed to be "busy" until it's done.
                    if (spindexerLogic.isFinished()) {
                        state = ManualPickupState.IDLE;
                    }
                    break;
            }
        }

        /**
         * Checks if the manager is currently busy indexing.
         * @return true if the state is not IDLE.
         */
        public boolean isIndexing() {
            return state != ManualPickupState.IDLE;
        }

        public ManualPickupState getState() {
            return state;
        }
    }
    /**
     * Manages a TeleOp-specific pickup sequence for a single ball.
     * This manager is designed to be triggered by a button press. It activates the intake
     * and waits until the color sensor detects a ball. Once a ball is captured, it
     * automatically indexes the spindexer to the next compartment and then returns to an
     * idle state, ready for the next button press.
     *
     * This manager explicitly does NOT control the shooter motor.
     */
    public static class TeleOpPickupManager {

        public enum PickupState {
            IDLE,           // Waiting for a command
            START_PICKUP,   // Closing the door and starting the intake
            WAITING_FOR_BALL, // Intake is running, waiting for the color sensor
            INDEXING,       // Ball detected, moving the spindexer to the next slot
            WAIT_FOR_INDEX  // Waiting for the spindexer rotation to finish
        }

        private PickupState state = PickupState.IDLE;

        // Hardware and Logic Dependencies
        private final Door door;
        private final IntakeActive intakeActive;
        private final SpindexerIndexerLogic spindexerLogic;
        private final ColorSensor colorSensor;
        private final IndicatorLight indicator; // For visual feedback

        /**
         * Initializes the TeleOpPickupManager.
         * @param door The robot's door mechanism.
         * @param intakeActive The robot's intake motor.
         * @param spindexerLogic The logic controller for the spindexer.
         * @param colorSensor The color sensor used to detect a ball.
         * @param indicator The indicator light for visual feedback.
         */
        public TeleOpPickupManager(Door door, IntakeActive intakeActive, SpindexerIndexerLogic spindexerLogic, ColorSensor colorSensor, IndicatorLight indicator) {
            this.door = door;
            this.intakeActive = intakeActive;
            this.spindexerLogic = spindexerLogic;
            this.colorSensor = colorSensor;
            this.indicator = indicator;
        }

        /**
         * The main update loop. Call this every cycle from your TeleOp.
         * @param pickupButtonPressed A rising-edge signal (wasJustPressed) to start the sequence.
         */
        public void update(boolean pickupButtonPressed) {
            // Always update the spindexer's internal logic.
            spindexerLogic.update();

            switch (state) {
                case IDLE:
                    // If the button is pressed and we're not already doing something...
                    if (pickupButtonPressed) {
                        state = PickupState.START_PICKUP;
                    }
                    break;

                case START_PICKUP:
                    // 1. Prepare for intake.
                    door.unlock();
                    door.forceClose();
                    intakeActive.intakeOn();
                    // 2. Move to the next state to wait for the ball.
                    state = PickupState.WAITING_FOR_BALL;
                    break;

                case WAITING_FOR_BALL:
                    // 3. Continuously check for a ball.
                    // The isBall method also handles the indicator light.
                    if (Utilities.isBall(colorSensor, indicator)) {
                        // 4. Ball detected! Turn off the intake and command the indexer.
                        intakeActive.intakeOff();
                        spindexerLogic.nextCompartment();
                        state = PickupState.INDEXING;
                    }
                    break;

                case INDEXING:
                    // This is a transition state. Wait for the spindexer to START moving.
                    // This prevents us from immediately skipping the wait state.
                    if (spindexerLogic.isBusy()) {
                        state = PickupState.WAIT_FOR_INDEX;
                    }
                    break;

                case WAIT_FOR_INDEX:
                    // 5. Wait for the spindexer to finish its rotation.
                    if (spindexerLogic.isFinished()) {
                        // 6. Sequence complete. Return to idle.
                        state = PickupState.IDLE;
                    }
                    break;
            }
        }

        /**
         * Call this to forcibly stop the pickup process and turn off the intake.
         * Useful if the button is released mid-sequence.
         */
        public void stop() {
            intakeActive.intakeOff();
            state = PickupState.IDLE;
        }

        /**
         * Checks if the manager is currently busy with a pickup sequence.
         * @return true if the state is not IDLE.
         */
        public boolean isBusy() {
            return state != PickupState.IDLE;
        }

        public PickupState getState() {
            return state;
        }
    }
}