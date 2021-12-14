package org.firstinspires.ftc.teamcode.math;

import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;

public class Point {

    public double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point() {

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

    public Point scalar(double scaleFactor) {
        return new Point(x * scaleFactor, y * scaleFactor);
    }

    public Point add(Point point) {
        return new Point(x+point.x, y+point.y);
    }

    public Point subtract(Point point) {
        return new Point(x-point.x, y-point.y);
    }

    public double dot(Point other) {
        return x * other.x + y * other.y;
    }

    public double getY() {
        return y;
    }

    public double getX() {
        return x;
    }

    public double hypot() { // Negated Y
        return Math.hypot(x, -y);
    }

    public static boolean inRange(Point currPoint, Point endPoint, double range) {
         double dist = getDistance(currPoint, endPoint);

         return dist < range;
    }

    public double atan2() { // Inverted and Negated Y
        return Math.atan2(x, -y);
    }

    public static double getDistance(Point start, Point end) {
        return Math.sqrt(Math.pow((end.getX()-start.getX()),2)+Math.pow((end.getY()-start.getY()),2));
    }

    public static boolean inRange(double var, double constant, double range) {
        if(constant-range<=var && constant+range>=var) {
            return true;
        }
        else {
            return false;
        }
    }

    public String toString() {
        return String.format(
                roundPlaces(x, 1) +
                        " " + roundPlaces(y, 1));
    }
}
