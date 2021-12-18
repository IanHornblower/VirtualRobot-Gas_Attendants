package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;

public class PurePursuit {

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
