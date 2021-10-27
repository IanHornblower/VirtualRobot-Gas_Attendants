package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Array;

import java.util.ArrayList;

// TODO: Fix absolute mess of a file

public class Trajectory {

    Pose2D[] path = new Pose2D[]{};

    CornettCore motionProfile;

    public Trajectory (Robot robot) {
        this.motionProfile = new CornettCore(robot);
    }

    public Pose2D[] get() {
        return path;
    }

    public void addWaypoint(Pose2D waypoint) {
        Array.appendArray(path, waypoint);
    }

    public void addRotation(double angle) {                                                             // May not work (get previous pose form pose array and
        Array.appendArray(path, new Pose2D(path[path.length-1].x, path[path.length-1].y, angle));       // creates new pose with same xy vector, but new angle
    }

    public void forward(double distance, Robot.controlType style) {
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(path[path.length-1].x, path[path.length-1].y+distance, path[path.length-1].heading);
                Array.appendArray(path, point);
                break;
            case ROBOT:
                double x = distance * Math.cos(path[path.length-1].heading);
                double y = distance * Math.sin(path[path.length-1].heading);
                point = new Pose2D(path[path.length-1].x + x, path[path.length-1].y + y, path[path.length-1].heading);
                Array.appendArray(path, point);
                break;
        }
    }

    public void backward(double distance, Robot.controlType style) {
        distance *= -1;
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(path[path.length-1].x, path[path.length-1].y-distance, path[path.length-1].heading);
                Array.appendArray(path, point);
                break;
            case ROBOT:
                double x = -distance * Math.cos(path[path.length-1].heading);
                double y = -distance * Math.sin(path[path.length-1].heading);
                point = new Pose2D(path[path.length-1].x + x, path[path.length-1].y + y, path[path.length-1].heading);
                Array.appendArray(path, point);
                break;
        }
    }

    public void right(double distance, Robot.controlType style) {
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(path[path.length-1].x + distance, path[path.length-1].y, path[path.length-1].heading);
                Array.appendArray(path, point);
                break;
            case ROBOT:
                double x = distance * Math.cos(path[path.length-1].heading - Math.PI/2.0);
                double y = distance * Math.sin(path[path.length-1].heading - Math.PI/2.0);
                point = new Pose2D(path[path.length-1].x + x, path[path.length-1].y + y, path[path.length-1].heading);
                Array.appendArray(path, point);
                break;
        }
    }

    public void left(double distance, Robot.controlType style) {
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(path[path.length-1].x, path[path.length-1].y+distance, path[path.length-1].heading);
                Array.appendArray(path, point);
                break;
            case ROBOT:
                double x = distance * Math.cos(path[path.length-1].heading+Math.PI/2.0);
                double y = distance * Math.sin(path[path.length-1].heading+Math.PI/2.0);
                point = new Pose2D(path[path.length-1].x + x, path[path.length-1].y + y, path[path.length-1].heading);
                Array.appendArray(path, point);
                break;
        }
    }

    public void runPath(double allowableDistanceError) throws InterruptedException {
        double pathLength = path.length;
        for(int i = 0; i < pathLength; i++) {
            motionProfile.runToPositionSync(path[i].x, path[i].y, path[i].heading, allowableDistanceError);
        }
    }

}