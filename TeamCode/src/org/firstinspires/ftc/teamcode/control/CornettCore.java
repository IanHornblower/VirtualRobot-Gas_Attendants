package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import jdk.jfr.internal.consumer.ChunkHeader;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.MiniPID;
import virtual_robot.controller.BotConfig;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Future;

public class CornettCore extends OpMode {

    private Robot robot;

    MiniPID defaultTurnPID = new MiniPID(2, 0, 0);

    double defaultTurnOutputMultiplier = 1;

    MiniPID defaultXPID = new MiniPID(1, 0,0);
    MiniPID defaultYPID = new MiniPID(1, 0,0);
    MiniPID defaultHeadingPID = new MiniPID(2, 0,0);

    double defaultXControlPointMultiplier = 3;
    double defaultYControlPointMultiplier = 3;
    double defaultHeadingControlPointMultiplier = 1.5;

    private double zero = 1e-9;
    boolean init = false;
    double distance = 0;

    public double output = 0, direction = 0, pidOutput = 0;

    public CornettCore(Robot robot) {
        this.robot = robot;
    }

    // TODO: Set up Trajectory Following, simple path following (circle around target), Later Implement Pure Pursuit (Circle Around Robot)
    // TODO: When running Sync functions allow for heading to finish turning (Maybe not, not an issue right now)

    public void rotateIMURaw(double heading, MiniPID turnPID, double outputMultiplier) {
        turnPID.setSetpoint(heading);
        turnPID.setOutputLimits(-1, 1);

        turnPID.setError(Curve.getShortestDistance(robot.IMU.getIMUHeading(), heading));

        direction = Curve.getDirection(robot.IMU.getIMUHeading(), heading);
        pidOutput = turnPID.getOutput(AngleUtil.deNormalizeAngle(robot.IMU.getIMUHeading()));
        output = direction * pidOutput * outputMultiplier;

        robot.DriveTrain.setMotorPowers(0, 0, output);
    }

    public void rotateIMU(double heading) {
        rotateIMURaw(heading, defaultTurnPID, defaultTurnOutputMultiplier);
    }

    public void rotateRaw(double heading, MiniPID turnPID, double outputMultiplier) {
        turnPID.setSetpoint(heading);
        turnPID.setOutputLimits(-1, 1);

        turnPID.setError(Curve.getShortestDistance(robot.pos.getHeading(), heading));

        direction = Curve.getDirection(robot.pos.getHeading(), heading);
        pidOutput = turnPID.getOutput(AngleUtil.deNormalizeAngle(robot.pos.getHeading()));
        output = direction * pidOutput * outputMultiplier;

        robot.DriveTrain.setMotorPowers(0, 0, output);
    }

    public void rotate(double heading) {
        rotateRaw(heading, defaultTurnPID, defaultTurnOutputMultiplier);
    }

    public void rotateSyncRaw(double heading, double anglePrecision, MiniPID turnPID, double turnOutputMultiplier) {
        distance = Curve.getShortestDistance(heading, anglePrecision);

        do {
            robot.updateOdometry();
            distance = Curve.getShortestDistance(robot.pos.getHeading(), heading);
            rotateRaw(heading, turnPID, turnOutputMultiplier);
        } while(distance > anglePrecision);
    }

    public void rotateSync(double heading, double anglePrecision) {
            rotateSyncRaw(heading, anglePrecision, defaultTurnPID, defaultTurnOutputMultiplier);
    }

    public void runToPositionRaw(double x, double y, double heading, MiniPID xPID,
                                 MiniPID yPID, MiniPID headingPID,
                                 double xControlPointMultiplier, double yControlPointMultiplier, double headingControlPointMultiplier)
    {
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

        double xControlPoint = xRawPower * xPIDOutput * xControlPointMultiplier;
        double yControlPoint = yRawPower * yPIDOutput * yControlPointMultiplier;
        double headingControlPoint = direction * headingPIDOutput * headingControlPointMultiplier;

        robot.DriveTrain.driveFieldCentric(xControlPoint, yControlPoint, headingControlPoint);
    }

    public void runToPosition(double x, double y, double heading) throws InterruptedException {
        runToPositionRaw(
                x, y, heading,
                defaultXPID, defaultYPID, defaultHeadingPID,
                defaultXControlPointMultiplier, defaultYControlPointMultiplier, defaultHeadingControlPointMultiplier);
    }

    public synchronized void runToPositionSyncRaw(double x, double y, double heading, double allowableDistanceError,
                                                  MiniPID xPID, MiniPID yPID, MiniPID headingPID,
                                                  double xControlPointMultiplier, double yControlPointMultiplier, double headingControlPointMultiplier) throws InterruptedException {
        distance = robot.pos.getDistanceFrom(new Point(x, y));
        do {
            robot.updateOdometry();
            distance = robot.pos.getDistanceFrom(new Point(x, y));
            runToPositionRaw(
                    x, y, heading,
                    xPID, yPID, headingPID,
                    xControlPointMultiplier, yControlPointMultiplier, headingControlPointMultiplier);
        } while(distance > allowableDistanceError);
    }

    public synchronized void runToPositionSync(double x, double y, double heading, double allowableDistanceError) throws InterruptedException {
            runToPositionSyncRaw(
                    x, y, heading, allowableDistanceError,
                    defaultXPID, defaultYPID, defaultHeadingPID,
                    defaultXControlPointMultiplier, defaultYControlPointMultiplier, defaultHeadingControlPointMultiplier);
    }


    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}