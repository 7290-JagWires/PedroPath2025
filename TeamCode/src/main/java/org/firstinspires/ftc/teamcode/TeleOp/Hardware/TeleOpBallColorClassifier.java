package org.firstinspires.ftc.teamcode.TeleOp.Hardware;

public class TeleOpBallColorClassifier {

    public enum BallColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    // tune these with your readings
    //f stands for float - a java type that allows you to use decimals
    private static final float PURPLE_HUE_MIN = 210f;
    private static final float PURPLE_HUE_MAX = 245f;

    private static final float GREEN_HUE_MIN = 145f;
    private static final float GREEN_HUE_MAX = 185f;

    private static final float MIN_SAT = 0.3f;  // ignore very gray / washed-out
    private static final float MIN_VAL = 2.0f;  // ignore very dark

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
}

