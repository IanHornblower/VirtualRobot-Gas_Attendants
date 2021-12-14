/**
package org.firstinspires.ftc.teamcode.DEPRICATED;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class PurePursuit {

    public static double TRACKWIDTH = 15.5;
    public static double MAXVELO = 0.02;
    public static double MAXACCELERATION = 1;

    public static List<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        // This method was lifted from Team 11115 Gluten Free's code.

        double baX = linePoint2.getX() - linePoint1.getX();
        double baY = linePoint2.getY() - linePoint1.getY();
        double caX = circleCenter.getX() - linePoint1.getX();
        double caY = circleCenter.getY() - linePoint1.getY();

        double a = baX * baX + baY * baY;
        double bBy2 = baX * caX + baY * caY;
        double c = caX * caX + caY * caY - radius * radius;

        double pBy2 = bBy2 / a;
        double q = c / a;

        double disc = pBy2 * pBy2 - q;
        if (disc < 0) {
            return Collections.emptyList();
        }

        double tmpSqrt = Math.sqrt(disc);
        double abScalingFactor1 = -pBy2 + tmpSqrt;
        double abScalingFactor2 = -pBy2 - tmpSqrt;

        List<Point> allPoints = null;

        Point p1 = new Point(linePoint1.getX() - baX * abScalingFactor1, linePoint1.getY() - baY * abScalingFactor1);
        if (disc == 0) {
            allPoints = Collections.singletonList(p1);
        }

        if (allPoints == null) {
            Point p2 = new Point(linePoint1.getX() - baX * abScalingFactor2, linePoint1.getY() - baY * abScalingFactor2);
            allPoints = Arrays.asList(p1, p2);
        }

        double maxX = Math.max(linePoint1.getX(), linePoint2.getX());
        double maxY = Math.max(linePoint1.getY(), linePoint2.getY());
        double minX = Math.min(linePoint1.getX(), linePoint2.getX());
        double minY = Math.min(linePoint1.getY(), linePoint2.getY());

        List<Point> boundedPoints = new ArrayList<Point>();

        for (Point point : allPoints) {

            if (point.getX() <= maxX && point.getX() >= minX)
                if (point.getY() <= maxY && point.getY() >= minY)
                    boundedPoints.add(point);

        }

        return boundedPoints;
    }

    public static double[] getTurn(double curvature) {
        return new double[] {MAXVELO*(2+curvature*TRACKWIDTH)/2, MAXVELO*(2-curvature*TRACKWIDTH)/2};
    }

    public static double getCurvature(Robot robot, Point lookAhead) {
        robot.updateOdometry();
        robot.updateAcumulatedHeading();

        double halfPI = Math.PI/2;
        double angle = Math.toRadians(robot.accumulatedHeading);

        double a = -Math.tan(halfPI - angle);
        double b = 1;
        double c = -a * robot.pos.x - robot.pos.y;

        double curvature = Math.abs(a*lookAhead.x + b * lookAhead.y + c)/Math.sqrt(Math.pow(a, 2)+Math.pow(b, 2));

        double side = Math.signum(Math.sin(halfPI - angle)*(lookAhead.x-robot.pos.x) - Math.cos(halfPI - angle) * (lookAhead.y - robot.pos.y));

        return side * curvature;
    }

    public static Point getLookAheadPoint(ArrayList<Pose2D> path, Robot robot, double radius) {
        Point pointToFollow = path.get(0).toPoint();
        double t1;
        double t2;

        for (int i = 0; i < path.size() - 1; i++) {
            robot.updateAcumulatedHeading();
            robot.updateOdometry();

            Point E = path.get(i).toPoint();
            Point L = path.get(i + 1).toPoint();

            Point f = E.subtract(robot.pos.toPoint());
            Point d = L.subtract(E);

            double a = d.dot(d);
            double b = 2 * f.dot(d);
            double c = f.dot(f) - radius * radius;

            double discriminant = b * b - 4 * a * c;

            if (discriminant < 0) {
                // No Intersections
            }
            else {
                discriminant = Math.sqrt(discriminant);
                t1 = (-b - discriminant) / (2*a);
                t2 = (-b + discriminant) / (2*a);
                if(t1 >= 0 && t1 <= 1) {
                    pointToFollow = E.add(new Point(t1 * d.x, t1 * d.y));
                }
                if(t2 >= 0 && t2 <= 1) {
                    pointToFollow = E.add(new Point(t2 * d.x, t2 * d.y));
                }
            }
        }
        return pointToFollow;
    }

    public static Point getFollowPoint(ArrayList<Pose2D> path, Robot robot, double radius) {
        Point pointToFollow = path.get(0).toPoint();

        for(int i = 0; i < path.size() - 1; i++) {
            Point lineStart = path.get(i).toPoint();
            Point lineEnd = path.get(i+1).toPoint();

            List<Point> intersections = PurePursuit.lineCircleIntersection(robot.pos.toPoint(), radius, lineStart, lineEnd);

            double closestAngle = 1e+9;  // Large Number

            for (Point thisIntersection : intersections) {
                robot.updateAcumulatedHeading();
                robot.updateOdometry();
                double dx = thisIntersection.getX() - robot.pos.x;
                double dy = thisIntersection.getY() - robot.pos.y;

                double theta = Math.atan2(dy, dx);
                double relativeAngle = Math.abs(theta - robot.pos.heading); // Heading Might be wrong | Check if it has to be in -180 to 180 format

                if (relativeAngle < closestAngle) {
                    closestAngle = relativeAngle;
                    pointToFollow = thisIntersection;
                }
            }
        }
        return pointToFollow;
    }

    public static ArrayList<Pose2D> extendPath (ArrayList<Pose2D> path, double radius) {
        Point lastPoint = path.get(path.size()-1).toPoint();
        Point anteLastPoint = path.get(path.size()-2).toPoint();

        double dxPer = lastPoint.x - anteLastPoint.x;
        double dyPer = lastPoint.y - anteLastPoint.y;

        double theta = Math.atan2(dyPer, dxPer);

        double dx = radius * Math.cos(theta);
        double dy = radius * Math.sin(theta);

        Point dPoint = new Point(dx, dy);
        Point extension = lastPoint.add(dPoint);

        path.add(new Pose2D(extension, 0));

        return path;
    }
}
 */
