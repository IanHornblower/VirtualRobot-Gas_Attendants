package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuitUtil;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Array;

import java.awt.event.ItemListener;
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

    public Trajectory (Robot robot, ArrayList<Pose2D> path) {
        this.motionProfile = new CornettCore(robot);
        this.path = path;
        this.robot = robot;
    }

    public enum PATH_TYPE {BASIC, PURE_PURSUIT}

    public ArrayList<Pose2D> get() {
        return path;
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
        ArrayList<Pose2D> rev = Array.reversePose2DArray(path);
        rev.remove(0);

        Trajectory traj = new Trajectory(
                robot, this.end()
        );

        for(int i = 0; i < rev.size(); i++) {
            traj.addWaypoint(rev.get(i));
        }

        return traj;
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

    public void followPath(PATH_TYPE type, CornettCore.DIRECTION direction, double error) throws InterruptedException {
        switch (type) {
            case BASIC:
                double pathLength = path.size();
                for(int i = 0; i < pathLength; i++) {
                    motionProfile.runToPositionSync(path.get(i).getX(), path.get(i).getY(), path.get(i).getHeading(), error);
                }
                break;
        }
    }

    public void followPath(PATH_TYPE pathType, CornettCore.DIRECTION direction, double radius, double error) throws InterruptedException {
        switch (pathType) {
            case PURE_PURSUIT:
                ArrayList<Pose2D> extendedPath = PurePursuitUtil.extendPath(path, radius);

                double distance = robot.pos.getDistanceFrom(path.get(path.size() - 1));

                do {
                    robot.updateOdometry();
                    Point pointToFollow = PurePursuitUtil.getLookAheadPoint(extendedPath, robot, radius);

                    distance = robot.pos.getDistanceFrom(path.get(path.size() - 1));
                    motionProfile.differentialRunToPosition(direction, pointToFollow);


                    robot.telemetry.addData("dist", distance);
                    robot.telemetry.update();

                } while(distance > error+radius);
                robot.DriveTrain.stopDrive();
        }
    }
}