package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.util.MiniPID;


public class DriveTrain {
    private final double maximumMotorSpeed = 0.85;

    private double p = 0, i = 0, d = 0, f = 0;

    public static double flbr = 0;
    public static double frbl = 0;

    double controlPointX = 0;
    double controlPointY = 0;
    double controlPointTurn = 0;

    double adjustedAngle = Math.toRadians(0);

    Robot robot;
    private BNO055IMU gyro;

    public DriveTrain(Robot robot) {
        this.robot = robot;

        gyro = robot.getImu();
    }

    public void setMotorVelocitys(double x, double y, double turn) { // Where x,y,turn are Velocities
        MiniPID xPIDF = new MiniPID(0, 0, 0, 0);
        MiniPID yPIDF = new MiniPID(0, 0, 0, 0);
        MiniPID turnPIDF = new MiniPID(0, 0, 0, 0);



        setMotorPowers(controlPointX, controlPointY, controlPointTurn);
    }

    public void driveFieldCentric(double x, double y, double turn) {
        Point vector = new Point(x, y);

        calculatePosition(
                vector.hypot(),
                vector.atan2() - robot.pos.getHeading() + adjustedAngle,
                turn);
    }

    public void calculatePosition(double dr, double theta, double turn) {
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);

        double mx = cos*dr;
        double my = sin*dr;
        double mt = turn;

        setMotorPowers(mx, my, mt);
    }

    public void setMotorPowers(double x, double y, double turn) {  // Invert x,y
        double h = Math.hypot(-x, -y);
        double theta = Math.atan2(-y, -x) - Math.toRadians(45);

        // Might divide my sin(45) set set sin(45) to 1
        //double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

        double denominator = 1;

        double[] motorVector = new double[] {
                (h * Math.cos(theta) - turn) / denominator,
                (h * Math.sin(theta) - turn) / denominator,
                (h * Math.sin(theta) + turn) / denominator,
                (h * Math.cos(theta) + turn) / denominator
        };

        setMotorPowers(motorVector);
    }

    public void setMotorPowers(double x, double y, double turn, double xyMag, double turnMag) {
        double h = Math.hypot(x, y);
        double theta = Math.atan2(y, x) - Math.toRadians(45);

        turn *= turnMag;  // Turn Speed
        h *= xyMag;       // xy, Speed

        // Might divide my sin(45) set set sin(45) to 1
        //double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

        double denominator = 1;

        double[] motorVector = new double[] {
                (h * Math.cos(theta) - turn) / denominator,
                (h * Math.sin(theta) - turn) / denominator,
                (h * Math.sin(theta) + turn) / denominator,
                (h * Math.cos(theta) + turn) / denominator
        };

        setMotorPowers(motorVector);
    }

    public void setMotorPowers(double fls, double bls, double frs, double brs) {
        robot.getFrontLeft().setPower(fls);
        robot.getFrontRight().setPower(frs);
        robot.getBackLeft().setPower(bls);
        robot.getBackRight().setPower(brs);
    }
    public void setMotorPowers(double[] powers) {
        setMotorPowers(
                powers[0],
                powers[1],
                powers[2],
                powers[3]
        );
    }
    public void setMotorPowers(double left, double right) {
        robot.getFrontLeft().setPower(left);
        robot.getBackLeft().setPower(left);
        robot.getFrontRight().setPower(right);
        robot.getBackRight().setPower(right);
    }
    public void stopDrive() {
        robot.getFrontLeft().setPower(0);
        robot.getBackLeft().setPower(0);
        robot.getFrontRight().setPower(0);
        robot.getBackRight().setPower(0);
    }

    public void setAngleOffset(double radians) {
        adjustedAngle = radians;
    }

    public DcMotor frontLeft() {
        return robot.getFrontLeft();
    }

    public DcMotor frontRight() {
        return robot.getFrontRight();
    }

    public DcMotor backLeft() {
        return robot.getBackLeft();
    }

    public DcMotor backRight() {
        return robot.getBackRight();
    }
}
