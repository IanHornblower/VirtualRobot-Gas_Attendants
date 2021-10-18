package org.firstinspires.ftc.teamcode.util;

public class MathUtil {

    public static double clipRange(double value) {
        return value <= -1 ? 1
                : value >= 1 ? 1
                : value;
    }

    public static double roundPlaces(double value, int places) {
        double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
    }

}
