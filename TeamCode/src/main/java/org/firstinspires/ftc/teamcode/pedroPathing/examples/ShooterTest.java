package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
@TeleOp()
public class ShooterTest extends LinearOpMode {

	static final double SHOOTER_VELOCITY_STOP = 0.0;
	static final double SHOOTER_VELOCITY_INCREMENT = 100.0;

	DcMotorEx shooterleft;
	double shooterVelocity = 0.0;
	boolean shooterOn;

	@Override
	public void runOpMode() {
		// Initialize the AprilTag webcam and pass it hardwareMap and telemetry for setup.

		// Wait for driver to press start

		shooterleft = (DcMotorEx) hardwareMap.dcMotor.get("shooter");

		shooterleft.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

		shooterleft.setVelocity(SHOOTER_VELOCITY_STOP);

		shooterleft.setDirection(DcMotorSimple.Direction.FORWARD);


		waitForStart();

		while (opModeIsActive()) {
			runShooters();
			showTelemetryInfo();
		}
	}

	void runShooters(){


		double shooterVelocityIncrement = SHOOTER_VELOCITY_INCREMENT;
		// if the user is pressing the "A" button...
		if (gamepad2.a) {
			// divide the SHOOTER_POWER_INCREMENT by 2 for finer control
			shooterVelocityIncrement = SHOOTER_VELOCITY_INCREMENT / 2;
		}
		// If the driver pressed the dpad down...
		if (gamepad2.dpadDownWasPressed()) {
			// subtract the preset velocity increment
			shooterVelocity = shooterVelocity - shooterVelocityIncrement;
			// If the driver pressed the dpad up...
		} else if (gamepad2.dpadUpWasPressed()) {
			// add the preset velocity increment
			shooterVelocity = shooterVelocity + shooterVelocityIncrement;
		}

		// If the driver pressed the "Y" button...
		if (gamepad2.yWasPressed()) {
			// if the shooterVelocity is zero...
			if (shooterVelocity == 0) {
				// set it to 1,000 for a default starting point
				shooterVelocity = 1000;
			}
			// toggle the shooterOn variable (turns it on if off, off is on)
			shooterOn = !shooterOn;
		}

		if (shooterOn) {
			setVelocities(shooterVelocity);
		} else {
			setVelocities(SHOOTER_VELOCITY_STOP);
		}
	}
	private void setVelocities(double velocityIn ){
		// velocity is ticks per second - it has a range of -2800 to 2800
		shooterleft.setVelocity( velocityIn );
	}

	void showTelemetryInfo(){
				telemetry.addLine("shooter is: " + (shooterOn ? "ON" : "OFF" ));
				telemetry.addLine("shooter velocity setting: " + shooterVelocity);
				telemetry.addLine( "left shooter velocity from motor: " + shooterleft.getVelocity() );
				telemetry.addLine( "gamepad: " + gamepad2 );
				telemetry.update();
	}
}
