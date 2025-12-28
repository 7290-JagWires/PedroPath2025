package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.IndicatorLight;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Door;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.IntakeActive;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Logic.SpindexerIndexerLogic;

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

        // Constructor to receive all hardware and logic components
        public PickupManager(Door door, Shooter shooter, IntakeActive intakeActive, SpindexerIndexerLogic spindexerLogic, ColorSensor colorSensor, IndicatorLight indicator) {
            this.door = door;
            this.shooter = shooter;
            this.intakeActive = intakeActive;
            this.spindexerLogic = spindexerLogic;
            this.colorSensor = colorSensor;
            this.indicator = indicator;
        }

        // Renamed from startPickup()
        public void start() {
            pickupState = PickupState.SPIN_UP;
            door.unlock();
            door.forceClose();
            pickupCount = 0;
            shooter.stop();
        }

        // Renamed from pickupItemsSM()
        public void update() {
            switch (pickupState) {
                case IDLE:
                    break;
                case SPIN_UP:
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
                                spindexerLogic.nextCompartmentAuto();
                                pickupState = PickupState.WAITING_FOR_SPIN;
                            } else {
                                totalBallCount = 0;
                                pickupState = PickupState.FINISHED;
                            }
                        }
                    }
                    break;
                case WAITING_FOR_SPIN:
                    if (spindexerLogic.spindexerLimitSwitchCheckPickup()) {
                        pickupTimer.reset();
                        if (pickupCount >= 1) {
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

        // Getter methods to check the state from the OpMode
        public PickupState getState() {
            return pickupState;
        }

        public boolean isFinished() {
            return pickupState == PickupState.FINISHED;
        }

        public void setTotalBallCount(int count) {
            this.totalBallCount = count;
        }

        public int getTotalBallCount() {
            return this.totalBallCount;
        }
    }

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

        // Constants
        private static final int SHOOTER_SPINUP_MS = 1500;
        private static final int LAUNCH_DELAY_MS = 500;
        private static final int TOTAL_BALLS_TO_DUMP = 3;

        // Constructor to receive all hardware components
        public DumpManager(Shooter shooter, Door door, SpindexerIndexerLogic spindexerLogic) {
            this.shooter = shooter;
            this.door = door;
            this.spindexerLogic = spindexerLogic;
        }

        /**
         * Initiates the automated process of dumping balls.
         */
        public void start() {
            if (dumpState == DumpState.IDLE || dumpState == DumpState.FINISHED) {
                this.dumpState = DumpState.SPIN_UP;
                this.dumpCount = 0;
                this.dumpTimer.reset();
                this.shooter.run();
            }
        }

        /**
         * Manages the state machine for dumping balls. Call this in a loop.
         */
        public void update() {
            switch (dumpState) {
                case IDLE:
                    break;
                case SPIN_UP:
                    if (dumpTimer.milliseconds() >= SHOOTER_SPINUP_MS) {
                        launchTimer.reset();
                        dumpState = DumpState.DUMPING;
                    }
                    break;
                case DUMPING:
                    door.forceOpenLock();
                    if (launchTimer.milliseconds() > LAUNCH_DELAY_MS) {
                        spindexerLogic.nextCompartmentAuto();
                    }
                    if (spindexerLogic.spindexerLimitSwitchCheck()) {
                        dumpCount++;
                        launchTimer.reset();
                        if (dumpCount >= TOTAL_BALLS_TO_DUMP) {
                            dumpState = DumpState.FINISHED;
                        }
                    }
                    break;
                case FINISHED:
                    // Actions to take once all balls are dumped
                    shooter.stop();
                    door.unlock();
                    door.forceClose();
                    break;
            }
        }

        // Getter methods to check the state from the OpMode
        public DumpState getState() {
            return dumpState;
        }

        public boolean isFinished() {
            return dumpState == DumpState.FINISHED;
        }

        public int getDumpCount() {
            return dumpCount;
        }
    }

}
