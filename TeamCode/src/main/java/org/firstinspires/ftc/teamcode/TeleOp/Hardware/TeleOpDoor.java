package org.firstinspires.ftc.teamcode.TeleOp.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleOpDoor {

    private final LinearOpMode opMode;
    private final Servo door;

    private final double DOOR_OPEN = 0.50;
    private final double DOOR_CLOSED = 0.25;

    private boolean prevY = false;

    // Used during dump all balls. When true, Y button cannot close the door.
    private boolean lockedOpen = false;

    public TeleOpDoor(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;
        door = hardwareMap.get(Servo.class, "door");
        door.setPosition(DOOR_CLOSED);
    }

    public void openDoor() {
        door.setPosition(DOOR_OPEN);
    }

    public void closeDoor() {
        if (!lockedOpen) {
            door.setPosition(DOOR_CLOSED);
        }
    }

    // Allows TeleOp to force the door closed even if toggle is off
    public void forceClose() {
        lockedOpen = false;
        closeDoor();
    }

    // Used during dump all balls so door stays open the whole time
    public void forceOpenLock() {
        lockedOpen = true;
        door.setPosition(DOOR_OPEN);
    }

    // Used after dump-all so door can be toggled normally again
    public void unlock() {
        lockedOpen = false;
    }

    public void toggleDoor() {
        double midPoint = (DOOR_OPEN + DOOR_CLOSED) / 2.0;

        if (door.getPosition() < midPoint) openDoor();
        else closeDoor();
    }

    public void run() {
        boolean y = opMode.gamepad2.y;

        if (y && !prevY && !lockedOpen) {
            toggleDoor();
        }

        prevY = y;
    }
}
