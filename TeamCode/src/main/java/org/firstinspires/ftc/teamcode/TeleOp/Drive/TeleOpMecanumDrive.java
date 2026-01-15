package org.firstinspires.ftc.teamcode.TeleOp.Drive;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.TeleOp.Hardware.TeleOpDriveTrain;

/**
 * MecanumDrive subsystem — supports both robot-centric and field-centric control.
 */
public class TeleOpMecanumDrive {
    private final TeleOpDriveTrain driveTrain;
    private final BNO055IMU imu;
    private boolean fieldCentric = false; // toggle state
    private double driveScale = 0.65;

    // ---------------- PINPOINT HEADING SUPPORT ----------------
    private double pinpointHeading = 0;

    /**
     * Called from TeleOp to update the heading used for field-centric drive.
     */
    public void setHeading(double headingRadians) {
        this.pinpointHeading = headingRadians;
    }

    public TeleOpMecanumDrive(TeleOpDriveTrain driveTrain, BNO055IMU imu) {
        this.driveTrain = driveTrain;
        this.imu = imu;
    }

    public TeleOpMecanumDrive(TeleOpDriveTrain driveTrain) {
        this(driveTrain, null);  // calls the main constructor with imu = null
    }

    /**
     * Toggle between field-centric and robot-centric control.
     */
    public void toggleMode() {
        fieldCentric = !fieldCentric;
    }

    /**
     * Returns the current drive mode name.
     */
    public String getModeName() {
        return fieldCentric ? "Field Centric" : "Robot Centric";
    }

    /**
     * Drives the robot using mecanum wheel math.
     * @param x        Strafe input (-1 to 1)
     * @param y        Forward/back input (-1 to 1)
     * @param rotation Rotation input (-1 to 1)
     */
    public void drive(double x, double y, double rotation) {
        // Invert Y-axis (stick forward drives forward)
        y = -y;

        // --- FIELD-CENTRIC TRANSFORM (Pinpoint heading) ---
        if (fieldCentric) {
            double headingRadians = pinpointHeading;

            double tempX = x * Math.cos(headingRadians) - y * Math.sin(headingRadians);
            double tempY = x * Math.sin(headingRadians) + y * Math.cos(headingRadians);

            x = tempX;
            y = tempY;
        }

        // --- MECANUM DRIVE FORMULAS ---
        double lf = y - x - rotation;
        double rf = y + x + rotation;
        double lr = y + x - rotation;
        double rr = y - x + rotation;

        // --- NORMALIZE ---
        double max = Math.max(Math.abs(lf),
                Math.max(Math.abs(rf), Math.max(Math.abs(lr), Math.abs(rr))));
        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lr /= max;
            rr /= max;
        }

        // --- APPLY POWER ---
        driveTrain.left_front.setPower(lf * driveScale);
        driveTrain.right_front.setPower(rf * driveScale);
        driveTrain.left_rear.setPower(lr * driveScale);
        driveTrain.right_rear.setPower(rr * driveScale);
    }

    /**
     * Stop all drive motors.
     */
    public void stop() {
        driveTrain.left_front.setPower(0);
        driveTrain.right_front.setPower(0);
        driveTrain.left_rear.setPower(0);
        driveTrain.right_rear.setPower(0);
    }
}
