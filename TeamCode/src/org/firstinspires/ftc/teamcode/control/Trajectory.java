package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Array;
import org.w3c.dom.html.HTMLBRElement;

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

    public Trajectory (Robot robot, ArrayList<Point> path, double angle) {
        this.motionProfile = new CornettCore(robot);
        this.path = toPoseArray(path, angle);
        this.robot = robot;
    }

    public Trajectory (Robot robot, ArrayList<Pose2D> path) {
        this.motionProfile = new CornettCore(robot);
        this.path = path;
        this.robot = robot;
    }

    public enum PATH_TYPE {BASIC, PURE_PURSUIT}

    public enum SPACIAL_TYPE {DISTANCE, PERCENTAGE}

    public double getDistance() {
        return 0;
    }

    public ArrayList<Pose2D> getPoseArray() {
        return path;
    }

    public ArrayList<Point> getPointArray() {
        ArrayList<Point> pointPath = new ArrayList<>();

        for(Pose2D point : path) {
            pointPath.add(point.toPoint());
        }

        return pointPath;
    };

    public ArrayList<Pose2D> toPoseArray(ArrayList<Point> path, double angle) {
        ArrayList<Pose2D> posePath = new ArrayList<>();

        for(Point pose : path) {
            posePath.add(new Pose2D(pose.x, pose.y, angle));
        }

        return posePath;
    }

    public Pose2D end() {
        return path.get(path.size()-1);
    }

    public void addWaypoint(Pose2D waypoint) {
        path.add(waypoint);
    }

    public void addWaypoint(Point waypoint) {
        path.add(new Pose2D(waypoint.x, waypoint.y, 0));
    }

    public Trajectory retrace() {
        ArrayList<Pose2D> oldPath = path;

        path = Array.reversePose2DArray(oldPath);
        return new Trajectory(robot, path);
    }

    public Trajectory at(ArrayList<Function> list) {
        for(int i = 0; i < list.size(); i++) {
            int finalI = i;
            Thread t1 = new Thread(new Runnable() {
                public void run() {
                    while(robot.accumulatedDistance <= list.get(finalI).distance) {
                        robot.pass();
                    }
                    list.get(finalI).execute();
                }
            });
            t1.start();
        }
        return this;
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

    public Trajectory followPath(PATH_TYPE type, double error) throws InterruptedException {
        switch (type) {
            case BASIC:
                double pathLength = path.size();
                for(int i = 0; i < pathLength; i++) {
                    motionProfile.runToPositionSync(path.get(i).getX(), path.get(i).getY(), path.get(i).getHeading(), error);
                }
                robot.DriveTrain.stopDrive();
        }
        return this;
    }

    public Trajectory followPath(PATH_TYPE pathType, CornettCore.DIRECTION direction, double radius, double error) throws InterruptedException {
        switch (pathType) {
            case PURE_PURSUIT:
                ArrayList<Pose2D> extendedPath = PurePursuit.extendPath(path, radius);

                double distance = robot.pos.getDistanceFrom(path.get(path.size() - 1));

                do {
                    robot.updateOdometry();
                    Point pointToFollow = PurePursuit.getLookAheadPoint(extendedPath, robot, radius);

                    distance = robot.pos.getDistanceFrom(path.get(path.size() - 1));
                    motionProfile.differentialRunToPosition(direction, pointToFollow);

                } while(distance > error+radius);
                robot.DriveTrain.stopDrive();
        }
        return this;
    }
}