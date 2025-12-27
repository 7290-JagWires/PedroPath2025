package org.firstinspires.ftc.teamcode.pedroPathing.Sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class ColorSensor {

    public enum BallColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }


    private final com.qualcomm.robotcore.hardware.ColorSensor sensor;

    private int red, green, blue, alpha;
    private double distanceCm;
    private boolean ballPresent;
    private BallColor detectedColor = BallColor.UNKNOWN;

    // tune these with your readings
    //f stands for float - a java type that allows you to use decimals
    private static final float PURPLE_HUE_MIN = 210f;
    private static final float PURPLE_HUE_MAX = 245f;

    private static final float GREEN_HUE_MIN = 145f;
    private static final float GREEN_HUE_MAX = 185f;

    private static final float MIN_SAT = 0.3f;  // ignore very gray / washed-out
    private static final float MIN_VAL = 2.0f;  // ignore very dark


    public ColorSensor(HardwareMap hwMap, String name) {
        sensor = hwMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, name);
    }

    public void update() {
        red = sensor.red();
        green = sensor.green();
        blue = sensor.blue();
        alpha = sensor.alpha();

        distanceCm =
                ((DistanceSensor) sensor)
                        .getDistance(DistanceUnit.CM);

        ballPresent = distanceCm < 4.0 || alpha > 150;

        detectedColor = ballPresent
                ? classify(red, green, blue)
                : BallColor.UNKNOWN;
    }

    public static BallColor classify(int r, int g, int b) {
        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(r, g, b, hsv);
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        if (sat < MIN_SAT || val < MIN_VAL) {
            return BallColor.UNKNOWN;
        }

        if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
            return BallColor.PURPLE;
        }

        if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
            return BallColor.GREEN;
        }

        return BallColor.UNKNOWN;
    }

//    public BallColor getBallColor() {
//        return classify(sensor.red(), sensor.green(), sensor.blue());
//    }


    public BallColor getBallColor() {
        return detectedColor;
    }

    public boolean isBallPresent() {
        return ballPresent;
    }

    public int getRed() { return red; }
    public int getGreen() { return green; }
    public int getBlue() { return blue; }
    public int getAlpha() { return alpha; }
    public double getDistanceCm() { return distanceCm; }
}

