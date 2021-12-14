package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.MiniPID;

public class CornettCore extends OpMode {

    private Robot robot;

    public enum DIRECTION {FORWARD, BACKWARD};

    MiniPID defaultTurnPID = new MiniPID(2, 0, 0);

    double defaultTurnOutputMultiplier = 1;

    MiniPID defaultXPID = new MiniPID(1, 0,0);
    MiniPID defaultYPID = new MiniPID(1, 0,0);
    MiniPID defaultHeadingPID = new MiniPID(5, 0,0);

    double defaultXControlPointMultiplier = 1;
    double defaultYControlPointMultiplier = 1;
    double defaultHeadingControlPointMultiplier = 1;

    private double zero = 1e-9;
    boolean init = false;
    double distance = 0;
    double angleDistance = 0;

    public static double Dp = 0.07;
    public static double ACp = 10;

    public double output = 0, direction = 0, turnPIDOutput = 0;

    public double xPIDOutput, yPIDOutput, headingPIDOutput;

    public CornettCore(Robot robot) {
        this.robot = robot;
    }

    // TODO: Set up Trajectory Following, simple path following (circle around target), Later Implement Pure Pursuit (Circle Around Robot)
    // TODO: When running Sync functions allow for heading to finish turning (Maybe not, not an issue right now)

    public void tuneTrackWidthIMU (double heading, double direction) {
        robot.updateAcumulatedHeading();
        MiniPID turnPID = defaultTurnPID;

        turnPID.setSetpoint(heading);
        turnPID.setOutputLimits(-1, 1);

        turnPID.setError(Math.abs(heading - Math.toRadians(robot.IMU.getAccumulatedHeadingInDegrees())));

        turnPIDOutput = turnPID.getOutput(AngleUtil.deNormalizeAngle(robot.IMU.getAccumulatedHeadingInDegrees()));
        output = direction * turnPIDOutput * defaultTurnOutputMultiplier;

        robot.DriveTrain.setMotorPowers(0, 0, output);
    }

    public void rotateRaw(double heading, MiniPID turnPID, double outputMultiplier) {
        turnPID.setSetpoint(heading);
        turnPID.setOutputLimits(-1, 1);

        turnPID.setError(Curve.getShortestDistance(robot.pos.getHeading(), heading));

        direction = Curve.getDirection(robot.pos.getHeading(), heading);
        turnPIDOutput = turnPID.getOutput(AngleUtil.deNormalizeAngle(robot.pos.getHeading()));
        output = direction * turnPIDOutput * outputMultiplier;

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

        xPIDOutput = xPID.getOutput(robot.pos.x);
        yPIDOutput = yPID.getOutput(robot.pos.y);
        headingPIDOutput = headingPID.getOutput(AngleUtil.deNormalizeAngle(robot.pos.getHeading()));

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
        angleDistance = Curve.getShortestDistance(heading, robot.pos.getHeading());
        do {
            robot.updateOdometry();
            distance = robot.pos.getDistanceFrom(new Point(x, y));
            angleDistance = Curve.getShortestDistance(heading, robot.pos.getHeading());
            runToPositionRaw(
                    x, y, heading,
                    xPID, yPID, headingPID,
                    xControlPointMultiplier, yControlPointMultiplier, headingControlPointMultiplier);
        } while(distance > allowableDistanceError || angleDistance > Math.toRadians(allowableDistanceError));
    }

    public synchronized void runToPositionSync(double x, double y, double heading, double allowableDistanceError) throws InterruptedException {
            runToPositionSyncRaw(
                    x, y, heading, allowableDistanceError,
                    defaultXPID, defaultYPID, defaultHeadingPID,
                    defaultXControlPointMultiplier, defaultYControlPointMultiplier, defaultHeadingControlPointMultiplier);
    }

    double t = 0, f = 0;

    public void setDifMotorForward(double targetX, double targetY) {
        robot.updateOdometry();

        double Dp = 0.07;
        double ACp = 2.3;

        double xError = targetX - robot.pos.x;
        double yError = targetY - robot.pos.y;
        double theta = Math.atan2(yError,xError);

        double distance = Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));

        f = 0 - distance * Dp;  // Could be -distance but Zero is there to model proper P-Loop
        t = AngleUtil.angleWrap(theta - robot.pos.getHeading()) * ACp;

        double left = f + t;
        double right = f - t;

        robot.DriveTrain.setMotorPowers(left, right);
    }

    public void setDifMotorReverse(double targetX, double targetY) {
        robot.updateOdometry();

        double xError = targetX - robot.pos.x;
        double yError = targetY - robot.pos.y;
        double theta = Math.atan2(-yError,-xError);

        double distance = Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));

        f = 0 - distance * Dp;  // Could be -distance but Zero is there to model proper P-Loop
        t = AngleUtil.angleWrap(theta - robot.pos.getHeading()) * ACp;

        double left = -f + t;
        double right = -f - t;

        robot.DriveTrain.setMotorPowers(left, right);
    }

    public void differentialRunToPosition(DIRECTION direction, Point pos) {
        switch (direction) {
            case FORWARD:
                setDifMotorForward(pos.x, pos.y);
                break;
            case BACKWARD:
                setDifMotorReverse(pos.x, pos.y);
        }
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}