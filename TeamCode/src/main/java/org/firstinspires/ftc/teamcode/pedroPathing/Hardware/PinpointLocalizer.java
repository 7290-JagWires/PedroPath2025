package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointLocalizer {

    private final GoBildaPinpointDriver odo;

    public PinpointLocalizer(HardwareMap hardwareMap) {

        // Name "odo" must match your config name
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set offsets based on your measurements
        odo.setOffsets(-38.1, -47.625, DistanceUnit.MM);

        // Set pod type (4-Bar or Swingarm)
        odo.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set encoder directions
        // X pod (forward wheel) should increase when robot moves forward
        // Y pod (strafe wheel) should increase when robot moves left
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset position + recalibrate IMU
        odo.resetPosAndIMU();
    }

    /** Call this every loop */
    public void update() {
        odo.update();
    }

    /** Returns Pose2D (X, Y, Heading) */
    public Pose2D getPose() {
        return odo.getPosition();
    }

    public double getX(DistanceUnit unit) { return odo.getPosX(unit); }
    public double getY(DistanceUnit unit) { return odo.getPosY(unit); }

    // New overload: matches TeleOpMain usage
    public double getHeading(AngleUnit angleUnit) {
        return odo.getHeading(angleUnit);
    }
}
