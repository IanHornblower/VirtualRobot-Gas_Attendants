/**
package org.firstinspires.ftc.teamcode.DEPRICATED;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Array;
import org.firstinspires.ftc.teamcode.util.MiniPID;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.DEPRICATED.PurePursuit.*;

public class PurePursuitTrajectory {

    ArrayList<Waypoint> path = new ArrayList<>();

    Pose2D startPos;
    Robot robot;
    double targetVelocity;
    double turnCoeff;

    double leftVelo = 0;
    double rightVelo = 0;

    public PurePursuitTrajectory(Robot robot, Pose2D startPos, double targetVelocity, double turnCoeff) {
        this.startPos = startPos;
        this.robot = robot;
        this.targetVelocity = targetVelocity;
        this.turnCoeff = turnCoeff;

        path.add(new Waypoint(startPos.toPoint()));
    }

    public void addWaypoint(Waypoint waypoint) {
        path.add(waypoint);
    }

    public static ArrayList<Waypoint> convertToPointArray(double[][] path) {
        ArrayList<Waypoint> newPath = new ArrayList<>();

        for(int i = 0; i < path.length; i++) {
            newPath.add(new Waypoint( new Point(path[i][0],
                    path[i][1]
            )));
        }

        return newPath;
    }

    public static double[][] clone2dDoubleArray(double[][] arr) {
        double[][] clonedArr = new double[arr.length][arr[0].length];
        for(int i = 0; i < arr.length; i++) {
            clonedArr[i] = arr[i];
        }
        return clonedArr;
    }

    public static double[][] convertToDoubleArray(ArrayList<Waypoint> path) {
        double[][] arrayPath = new double[path.size()][2];
        for(int i = 0; i < path.size(); i++) {
            arrayPath[i][0] = path.get(i).pos.x;
            arrayPath[i][1] = path.get(i).pos.y;
        }
        return arrayPath;
    }

    public void interpolatePath(double spacing) {
        ArrayList<Waypoint> newPath = new ArrayList<>();
        int segments = path.size()-1;
        double initialRatio = 100 / ((spacing + 1) * 100);

        newPath.add(path.get(0));

        for(int i = 0; i <= segments-1; i++) {
            double ratio = 100 / ((spacing + 1) * 100);
            for(int j = 0; j < spacing + 1; j++) {                                                  // new point = current + ratio * (next - current)
                newPath.add(new Waypoint(path.get(i).pos.add(path.get(i+1).pos.subtract(path.get(i).pos).scalar(ratio))));    // current + (next - current) * ratio
                ratio += initialRatio;
            }
        }
    }

    public void smoothPath(double smoothing, double weight, double tolerance) {
        double[][] arrayPath = convertToDoubleArray(path);

        double[][] newPath = clone2dDoubleArray(arrayPath);
        double change = tolerance;

        while(change >= tolerance) {
            change = 0.0;
            for(int i = 1; i < arrayPath.length - 1; i++) {
                for(int j = 0; j < arrayPath[i].length; j++) {
                    double aux = newPath[i][j];

                    newPath[i][j] += smoothing*(arrayPath[i][j]-newPath[i][j]) + weight*(newPath[i-1][j]+newPath[i+1][j]-2*newPath[i][j]);

                    //newPath[i][j] += smoothing * (arrayPath[i][j] - newPath[i][j]) + weight * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
            }
        }

        path = convertToPointArray(newPath);
    }

    public void calculateDistance() {
        for(int i = 0; i < path.size()-1; i++) {
            path.get(i).distance = Point.getDistance(path.get(i).pos, path.get(i+1).pos);
        }
    }

    public void calculateCurvature() {
        for(int i = 0; i <= path.size()-1; i++) {
            path.get(i).pos.add(new Point(0.0001, 0.0001));
            double k1 = .5 * (Math.pow(path.get(i).pos.x, 2) + Math.pow(path.get(i).pos.y,2) - Math.pow(path.get(i-1).pos.x,2) - Math.pow(path.get(i-1).pos.y,2)) / (path.get(i).pos.x - path.get(i-1).pos.x);
            double k2 = (path.get(i).pos.y - path.get(i-1).pos.y) / (path.get(i).pos.x - path.get(i-1).pos.y);
            double b = .5 * (Math.pow(path.get(i-1).pos.x,2) - 2 * path.get(i-1).pos.x * k1 + Math.pow(path.get(i-1).pos.y,2) - Math.pow(path.get(i+1).pos.x,2) + 2 * path.get(i+1).pos.x * k1 - Math.pow(path.get(i+1).pos.y, 2)) / (path.get(i+1).pos.x * k2 - path.get(i+1).pos.y + path.get(-1).pos.y - path.get(i-1).pos.x * k2);
            double a = k1 - k2 * b;
            double r = Math.sqrt(Math.pow(path.get(i).pos.x - a, 2) + Math.pow(path.get(i).pos.y - a, 2));

            path.get(i).curvature = 1/r;
        }
    }

    public void calculateVelocity() {
        for(int i = 0; i < path.size()-1; i++) {
            path.get(i).targetVelocity = Math.min(targetVelocity, turnCoeff/path.get(i).curvature);
        }
    }

    public void calculateAccelerationLimit() {
        ArrayList<Waypoint> reversed = Array.reverseArrayList(path);
        reversed.remove(0);
        for(int i = 1; i < reversed.size(); i++) {

            //math.sqrt(smooth_waypoints[i-1][5]**2 + 2*float(config["VELOCITY"]["MAX_ACCEL"])*math.sqrt((w[0] - smooth_waypoints[i-1][0]) ** 2 + (w[1] - smooth_waypoints[i-1][1]) ** 2))

            double acceleration = Math.sqrt(Math.pow(path.get(i-1).accelrationLimit,2) + 2*MAXACCELERATION*Math.sqrt(Math.pow(reversed.get(i).pos.x - path.get(i-1).pos.x, 2) + Math.pow(reversed.get(i).pos.y - path.get(i-1).pos.y, 2)));

            if(acceleration < reversed.get(i).accelrationLimit) {
                reversed.get(i).accelrationLimit = acceleration;
            }
            else {
            }

            path.get(i).accelrationLimit = acceleration;
        }
    }

    public double closest(ArrayList<Waypoint> path) { // Returns index of path at where distance was the closest
        double[] initial = new double[]{0, robot.pos.getDistanceFrom(path.get(0).pos)};
        for(int i = 0; i < path.size(); i++ ) {
            double distance = robot.pos.getDistanceFrom(path.get(i).pos);
            if(distance < initial[1]) {
                initial = new double[]{i, distance};
            }
        }
        return initial[0];
    }

    public double[] calculateWheelVelocities(double curvature, double velocity) {
        //double velocity = (leftVelo + rightVelo) /2;
        //double angularVelocity = (leftVelo - rightVelo)/TRACKWIDTH+2;
        //velocity = angularVelocity/curvature;

        leftVelo = velocity * (2 + curvature*TRACKWIDTH)/2;
        rightVelo = velocity * (2 - curvature*TRACKWIDTH)/2;

        return new double[]{leftVelo, rightVelo};
    }

    public void buildTrajectory() {
        interpolatePath(4);
        smoothPath(0.9, 0.1, 0.132);
        calculateDistance();
        calculateCurvature();
        calculateVelocity();
        calculateAccelerationLimit();
    }

    public void setMotorVelocities(double[] velocities, double kV, double kA) {
        MiniPID left = new MiniPID(1, 0, 0, kV*kA);
        MiniPID right = new MiniPID(1, 0, 0, kV*kA);

        robot.updateEncoderVelocity();

        left.setSetpoint(velocities[0]);
        right.setSetpoint(velocities[1]);

        left.setError(velocities[0]-robot.leftVelocity);
        right.setError(velocities[1]-robot.rightVelocity);

        double leftVelo = left.getOutput();
        double rightVelo = right.getOutput();


        //double left = kV* [] + kA*[];


        robot.DriveTrain.setMotorPowers(leftVelo, rightVelo);
    }

    public void followTrajectory(double lookahead, double dick) {
        double curvature = 0;

        while(closest(path) != path.size()-1) {
            Point followPoint = getLookAheadWaypoint(path, robot, lookahead);
            int closest = (int)closest(path);
            if(0 > closest) {
                curvature = getCurvature(robot, followPoint);
            }
            else curvature = 0.00001;

            double velo = path.get(closest).distance;

            setMotorVelocities(calculateWheelVelocities(curvature, velo), 5, 2);
        }
    }
}
 */
