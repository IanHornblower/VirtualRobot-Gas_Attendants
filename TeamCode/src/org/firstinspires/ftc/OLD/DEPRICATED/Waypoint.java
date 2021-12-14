/**
package org.firstinspires.ftc.teamcode.DEPRICATED;

import org.firstinspires.ftc.teamcode.math.Point;

public class Waypoint {

    public Point pos;
    public double targetVelocity;
    public double curvature;
    public double distance;
    public double accelrationLimit;

    public Waypoint(Point pos) {
        this.pos = pos;
        this.targetVelocity = 1;
        this.accelrationLimit = 1;
        this.distance = 1;
        this.curvature = 1 ;
    }



    public void scalePose(double scaleFactor) {
        pos.x *= scaleFactor;
        pos.y *= scaleFactor;
    }

    public Waypoint scalar(double scaleFactor) {
        return new Waypoint(new Point(pos.x * scaleFactor, pos.y * scaleFactor));
    }

    public Point add(Point point) {
        return new Point(pos.x+point.x, pos.y+point.y);
    }

    public Point subtract(Point point) {
        return new Point(pos.x-point.x, pos.y-point.y);
    }

    public double dot(Point other) {
        return pos.x * other.x + pos.y * other.y;
    }

    public double hypot() { // Negated Y
        return Math.hypot(pos.x, pos.y);
    }

    public double atan2() { // Inverted and Negated Y
        return Math.atan2(pos.x, -pos.y);
    }

    public void setPos(Point pos) {
        this.pos = pos;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public void setCurvature(double curvature) {
        this.curvature = curvature;
    }

    public Point toPoint() {
        return pos;
    }

}
 */
