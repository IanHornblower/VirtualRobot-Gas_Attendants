package org.firstinspires.ftc.teamcode.util;

public class Controller {
    public static int LEFT_TRIGGER_Y_POW = 2;
    public static int LEFT_TRIGGER_X_POW = 2;

    public static double scale = 1;

    public static double deadZone(double control, double deadZone) {
        return Math.abs(control) > deadZone ? control : 0;
    }
}
