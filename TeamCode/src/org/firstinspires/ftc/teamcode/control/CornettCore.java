package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import jdk.jfr.internal.consumer.ChunkHeader;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.MiniPID;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Future;

public class CornettCore extends OpMode {

    private Robot robot;

    private double zero = 1e-9;
    boolean init = false;
    double distance = 0;

    private double x, y, allowableDistanceError;

    public double output = 0, direction = 0, pidOutput = 0;

    public CornettCore(Robot robot) {
        this.robot = robot;
    }

    // TODO: Set up Trajectory, simple path following (circle around target), Later Implement Pure Pursuit (Circle Around Robot)

    public void rotateRaw(double heading, double anglePrecision) {   // Note this is currently Async and does not wait for other functions to be called
        MiniPID turnPID = new MiniPID(2, 0, 0);          // This will be cancelled and run the later iteration of this function without wait
                                                                 //  Angle Precision/Tolerance Has yet to be setup | PID has to be defined somewhere else, DT maybe?
        turnPID.setSetpoint(heading);
        turnPID.setOutputLimits(-1, 1);

        turnPID.setError(Curve.getShortestDistance(robot.pos.getHeading(), heading));

        direction = Curve.getDirection(robot.pos.getHeading(), heading);
        pidOutput = turnPID.getOutput(AngleUtil.deNormalizeAngle(robot.pos.getHeading()));
        output = direction * pidOutput;

        robot.DriveTrain.setMotorPowers(0, 0, output);
    }

    public void rotate(double heading) {
        rotateRaw(heading,0);
    }

    public void runToPositionRaw(double x, double y, double heading, double allowableDistanceError) {
        MiniPID xPID = new MiniPID(0.3, 0, 0);
        MiniPID yPID = new MiniPID(0.3, 0, 0);
        MiniPID headingPID = new MiniPID(2, 0, 0);

        xPID.setSetpoint(x);
        yPID.setSetpoint(y);
        headingPID.setSetpoint(heading);

        xPID.setOutputLimits(-1, 1);
        yPID.setOutputLimits(-1, 1);
        headingPID.setOutputLimits(-1, 1);

        xPID.setError(Math.abs(x-robot.pos.x));
        yPID.setError(Math.abs(y-robot.pos.y));
        headingPID.setError(Curve.getShortestDistance(robot.pos.getHeading(), heading));

        double xPIDOutput = xPID.getOutput(robot.pos.x);
        double yPIDOutput = yPID.getOutput(robot.pos.y);
        double headingPIDOutput = headingPID.getOutput(AngleUtil.deNormalizeAngle(robot.pos.getHeading()));

        double theta = Curve.getAngle(robot.pos, new Point(x, y));

        direction = Curve.getDirection(robot.pos.getHeading(), heading);

        double xRawPower = Math.cos(theta);
        double yRawPower = Math.sin(theta);

        double xControlPoint = xRawPower * xPIDOutput;
        double yControlPoint = yRawPower * yPIDOutput;
        double headingControlPoint = direction * headingPIDOutput;

        robot.DriveTrain.driveFieldCentric(xControlPoint, yControlPoint, headingControlPoint);
        robot.pos.PIDSUM = Math.abs(xPIDOutput + yPIDOutput * headingPIDOutput);
    }

    public void runToPosition(double x, double y, double heading) throws InterruptedException {
        runToPositionRaw(x, y, heading, 1);
    }

    public void runToPositionSync(double x, double y, double heading, double allowableDistanceError) throws InterruptedException {
        distance = robot.pos.getDistanceFrom(new Point(x, y));
        do {
            robot.updateOdometry();
            distance = robot.pos.getDistanceFrom(new Point(x, y));
            runToPosition(x, y, heading);
        } while(distance > allowableDistanceError);
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
