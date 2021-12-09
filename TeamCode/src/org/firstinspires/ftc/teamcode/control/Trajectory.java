package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

import java.util.ArrayList;

public class Trajectory {

    CornettCore motionProfile;

    ArrayList<Pose2D> path = new ArrayList<>();

    Pose2D startPos;

    Robot robot;

    public Trajectory (Robot robot, Pose2D startPos) {
        this.motionProfile = new CornettCore(robot);
        this.startPos = startPos;
        this.robot = robot;

        path.add(startPos);
    }

    public enum PATH_TYPE {BASIC, PURE_PURSUIT, DIFFERENTIAL_PURE_PURSUIT}

    public ArrayList<Pose2D> get() {
        return path;
    }

    public Pose2D end() {
        return path.get(path.size()-1);
    }

    public void addWaypoint(Pose2D waypoint) {
        path.add(waypoint);
    }

    /**
     * Always use AngleUtil.interpretAngle(double angle), with this function.
     * @param angle
     */

    public void addRotation(double angle) {                                                             // May not work (get previous pose form pose array and
        path.add(new Pose2D(path.get(path.size()-1).x, path.get(path.size()-1).y, angle));       // creates new pose with same xy vector, but new angle
    }

    public void forward(double distance, Robot.controlType style) {
        Pose2D previous = path.get(path.size()-1);
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(path.get(path.size()-1).x, path.get(path.size()-1).y+distance, path.get(path.size()-1).heading);
                path.add(point);
                break;
            case ROBOT:
                double x = distance * Math.cos(AngleUtil.interpretRadians(previous.heading));
                double y = distance * Math.sin(AngleUtil.interpretRadians(previous.heading));
                point = new Pose2D(previous.x + x, previous.y + y, previous.heading);
                path.add(point);
                break;
        }
    }

    public void backward(double distance, Robot.controlType style) {
        Pose2D previous = path.get(path.size()-1);
        distance *= -1;
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(previous.x, previous.y-distance, previous.heading);
                path.add(point);
                break;
            case ROBOT:
                double x = distance * Math.cos(AngleUtil.interpretRadians(previous.heading));
                double y = distance * Math.sin(AngleUtil.interpretRadians(previous.heading));
                point = new Pose2D(previous.x + x, previous.y + y, previous.heading);
                path.add(point);
                break;
        }
    }

    public void right(double distance, Robot.controlType style) {
        Pose2D previous = path.get(path.size()-1);
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(previous.x + distance, previous.y, previous.heading);
                path.add(point);
                break;
            case ROBOT:
                double x = distance * Math.cos(AngleUtil.interpretRadians(previous.heading + Math.PI/2.0));
                double y = distance * Math.sin(AngleUtil.interpretRadians(previous.heading + Math.PI/2.0));
                point = new Pose2D(previous.x + x, previous.y + y, previous.heading);
                path.add(point);
                break;
        }
    }

    public void left(double distance, Robot.controlType style) {
        Pose2D previous = path.get(path.size()-1);
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(previous.x, previous.y+distance, previous.heading);
                path.add(point);
                break;
            case ROBOT:
                double x = distance * Math.cos(AngleUtil.interpretRadians(previous.heading - Math.PI/2.0));
                double y = distance * Math.sin(AngleUtil.interpretRadians(previous.heading - Math.PI/2.0));
                point = new Pose2D(previous.x + x, previous.y + y, previous.heading);
                path.add(point);
                break;
        }
    }

    public void followPath(PATH_TYPE type, double allowableDistanceError) throws InterruptedException {
        switch (type) {
            case BASIC:
                double pathLength = path.size();
                for(int i = 0; i < pathLength; i++) {
                    motionProfile.runToPositionSync(path.get(i).getX(), path.get(i).getY(), path.get(i).getHeading(), allowableDistanceError);
                }
                break;
            default:
                // Wrong Path Type - Try BASIC
        }
    }

    public void followPath(PATH_TYPE type, double radius, double allowableDistanceError) throws InterruptedException {
        switch (type) {
            case DIFFERENTIAL_PURE_PURSUIT:
                ArrayList<Pose2D> extendedPath = PurePursuit.extendPath(path, radius);

                double distance = robot.pos.getDistanceFrom(path.get(path.size() - 1));

                Point lastPoint = path.get(path.size()-1).toPoint();
                Point anteLastPoint = path.get(path.size()-2).toPoint();

                double dxPer = lastPoint.x - anteLastPoint.x;
                double dyPer = lastPoint.y - anteLastPoint.y;

                double tangent = Math.atan2(dyPer, dxPer);

                do {
                    robot.updateOdometry();
                    Point pointToFollow = PurePursuit.getLookAheadPoint(extendedPath, robot, radius);

                    double x = pointToFollow.getX(), y = pointToFollow.getY();
                    double dx = x - robot.pos.x, dy = y - robot.pos.y;
                    double theta = Math.atan2(dy, dx);
                    distance = robot.pos.getDistanceFrom(path.get(path.size() - 1));
                    //motionProfile.runToPosition(x, y, theta);
                    robot.DriveTrain.differentialRunToPosition(DriveTrain.DIRECTION.BACKWARD, pointToFollow);

                } while(distance > allowableDistanceError+radius);
                //motionProfile.rotateSync(tangent, Math.toRadians(allowableDistanceError));
                break;
            default:
                // Wrong Path Type - Try either Pure Pursuit Path types

        }
    }

    public void followPath(PATH_TYPE type, double radius, double tangent, double allowableDistanceError) throws InterruptedException {
        switch (type) {
            case PURE_PURSUIT:
                double pathLength = path.size();
                ArrayList<Pose2D> extendedPath = PurePursuit.extendPath(path, radius);

                double distance = robot.pos.getDistanceFrom(path.get(path.size() - 1));

                do {
                    Point pointToFollow = PurePursuit.getFollowPoint(extendedPath, robot, radius);

                    double x = pointToFollow.getX(), y = pointToFollow.getY();
                    double dx = x - robot.pos.x, dy = y - robot.pos.y;

                    distance = robot.pos.getDistanceFrom(path.get(path.size() - 1));
                    motionProfile.runToPosition(x, y, tangent);
                } while(distance > allowableDistanceError+radius);
                break;
            case BASIC: // DIF PP
                extendedPath = PurePursuit.extendPath(path, radius);

                do {
                    Point pointToFollow = PurePursuit.getFollowPoint(extendedPath, robot, radius);

                    double x = pointToFollow.getX(), y = pointToFollow.getY();
                    double dx = x - robot.pos.x, dy = y - robot.pos.y;
                    double theta = Math.atan2(dy, dx);
                    distance = robot.pos.getDistanceFrom(path.get(path.size() - 1));
                    motionProfile.runToPosition(x, y, theta);

                } while(distance > allowableDistanceError+radius);
                motionProfile.rotateSync(tangent, Math.toRadians(allowableDistanceError));
                break;
            default:
                // Wrong Path Type - Try either Pure Pursuit Path types

        }
    }
    public void testNewPP(double radius) throws InterruptedException {
        double pathLength = path.size();
        ArrayList<Pose2D> extendedPath = PurePursuit.extendPath(path, radius);

        double distance = robot.pos.getDistanceFrom(path.get(path.size() - 1));

        do {
            Point pointToFollow = PurePursuit.getFollowPoint(extendedPath, robot, radius);

            motionProfile.runToPosition(pointToFollow.x, pointToFollow.y, pointToFollow.atan2());


        } while(distance > 1+radius);
    }
}