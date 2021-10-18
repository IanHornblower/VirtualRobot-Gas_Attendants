package org.firstinspires.ftc.teamcode.math;

public class Angle {

    double current, end;

    public Angle(double current, double end) {
        this.current = current;
        this.end = end;
    }

    public void update(double current, double end) {
        this.current = current;
        this.end = end;
    }

    public double shortestDistance = Curve.getShortestDistance(current, end);

    public double direction = Curve.getDirection(current, end);

}
