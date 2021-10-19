package org.firstinspires.ftc.teamcode.math;

import static java.lang.Math.atan;
import static java.lang.Math.decrementExact;

public class Curve {

    public static double getShortestDistance(double currentAngle, double targetAngle) {  // Radians
        double x1=Math.cos(currentAngle), y1=Math.sin(currentAngle), x2=Math.cos(targetAngle), y2=Math.sin(targetAngle);

        return Math.acos(x1*x2 + y1*y2);
    }

    public static double getAngle(Point current, Point end) {
        double dx = end.getX()-current.getX();
        double dy = end.getY()-current.getY();
        return Math.atan2(dy, dx);
    }

    public static double getAngle(Pose2D current, Point end) {
        double dx = end.getX()-current.getX();
        double dy = end.getY()-current.getY();
        return Math.atan2(dy, dx);
    }

    public static int getDirection(double currentAngle, double targetAngle) {  // Radians --> Degrees --> Direction
        double difference = targetAngle - currentAngle;

        if(difference < 0) {
            difference += 2*Math.PI;
        }

        if(difference > Math.PI) {
            return 1;
        }

        else {
            return -1;
        }
    }
}
