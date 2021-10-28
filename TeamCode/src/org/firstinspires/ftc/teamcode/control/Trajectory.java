package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.AnalogInput;
import javafx.geometry.Pos;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Array;

import java.util.ArrayList;

// TODO: Fixed the mess, now the Robot Centric Controls need to be fine tuned.

public class Trajectory {

    CornettCore motionProfile;

    ArrayList<Pose2D> path = new ArrayList<>();

    Pose2D startPos;

    public Trajectory (Robot robot, Pose2D startPos) {
        this.motionProfile = new CornettCore(robot);
        this.startPos = startPos;

        path.add(startPos);
    }

    public ArrayList<Pose2D> get() {
        return path;
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
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(path.get(path.size()-1).x, path.get(path.size()-1).y+distance, path.get(path.size()-1).heading);
                path.add(point);
                break;
            case ROBOT:
                double x = distance * Math.cos(path.get(path.size()-1).heading);
                double y = distance * Math.sin(path.get(path.size()-1).heading);
                point = new Pose2D(path.get(path.size()-1).x + x, path.get(path.size()-1).y + y, path.get(path.size()-1).heading);
                path.add(point);
                break;
        }
    }

    public void backward(double distance, Robot.controlType style) {
        distance *= -1;
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(path.get(path.size()-1).x, path.get(path.size()-1).y-distance, path.get(path.size()-1).heading);
                path.add(point);
                break;
            case ROBOT:
                double x = -distance * Math.cos(path.get(path.size()-1).heading);
                double y = -distance * Math.sin(path.get(path.size()-1).heading);
                point = new Pose2D(path.get(path.size()-1).x + x, path.get(path.size()-1).y + y, path.get(path.size()-1).heading);
                path.add(point);
                break;
        }
    }

    public void right(double distance, Robot.controlType style) {
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(path.get(path.size()-1).x + distance, path.get(path.size()-1).y, path.get(path.size()-1).heading);
                path.add(point);
                break;
            case ROBOT:
                double x = distance * Math.cos(path.get(path.size()-1).heading - Math.PI/2.0);
                double y = distance * Math.sin(path.get(path.size()-1).heading - Math.PI/2.0);
                point = new Pose2D(path.get(path.size()-1).x + x, path.get(path.size()-1).y + y, path.get(path.size()-1).heading);
                path.add(point);
                break;
        }
    }

    public void left(double distance, Robot.controlType style) {
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(path.get(path.size()-1).x, path.get(path.size()-1).y+distance, path.get(path.size()-1).heading);
                path.add(point);
                break;
            case ROBOT:
                double x = distance * Math.cos(path.get(path.size()-1).heading+Math.PI/2.0);
                double y = distance * Math.sin(path.get(path.size()-1).heading+Math.PI/2.0);
                point = new Pose2D(path.get(path.size()-1).x + x, path.get(path.size()-1).y + y, path.get(path.size()-1).heading);
                path.add(point);
                break;
        }
    }

    public void runPath(double allowableDistanceError) throws InterruptedException {
        double pathLength = path.size();
        for(int i = 0; i < pathLength; i++) {
            motionProfile.runToPositionSync(path.get(i).getX(), path.get(i).getY(), path.get(i).getHeading(), allowableDistanceError);
        }
    }

}