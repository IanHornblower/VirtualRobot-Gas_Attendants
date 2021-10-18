package org.firstinspires.ftc.teamcode.math;

public class Line {

    private double getSegmentLength (Point start, Point end) {
        return Math.sqrt(Math.pow(end.getX() - start.getX(), 2) + Math.pow(end.getY() - start.getY(), 2));
    }
}
