package org.firstinspires.ftc.teamcode.math;

public class Point {

    private double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void invertPose() {
        double tempX = x, tempY = y;
        x = tempY;
        y = tempX;
    }

    public void scalePose(double scaleFactor) {
        x *= scaleFactor;
        y *= scaleFactor;
    }

    public double getY() {
        return y;
    }

    public double getX() {
        return x;
    }

    public double hypot() {
        return Math.hypot(x, y);
    }

    public double atan2() {
        return Math.atan2(y, x);
    }

    public static double getDistance(Point start, Point end) {
        return Math.sqrt(Math.pow((end.getX()-start.getX()),2)+Math.pow((end.getY()-start.getY()),2));
    }
}
