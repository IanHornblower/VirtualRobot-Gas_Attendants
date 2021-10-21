package org.firstinspires.ftc.teamcode.control;

import javafx.geometry.Pos;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Array;

import java.util.ArrayList;

public class Trajectory {

    Pose2D[] path = new Pose2D[0];

    public Trajectory () {
    }

    public void addWaypoint(Pose2D waypoint) {
        Array.appendArray(path, waypoint);
    }

    public void addRotation(double angle) {                                                             // May not work (get previous pose form pose array and
        Array.appendArray(path, new Pose2D(path[path.length-1].x, path[path.length-1].y, angle));       // creates new pose with same xy vector, but new angle
    }

    public void runPath(double a) {
        double pathLength = path.length;
        for(int i = 0; i < pathLength; i++) {

        }
    }

}