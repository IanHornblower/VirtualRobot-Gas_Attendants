package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class PurePursuit {

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
