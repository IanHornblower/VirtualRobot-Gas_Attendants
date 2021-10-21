package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

public class Robot {
    private DcMotor backLeft, backRight, frontLeft, frontRight;

    private DcMotorEx leftEncoder, rightEncoder, lateralEncoder;

    private BNO055IMU imu;
    private Orientation angles;

    public HardwareMap hwMap;
    public DriveTrain DriveTrain;
    public IMU IMU;

    // Robot Kinematics

    // Odmometric Constraints
    final static double L = 12;  // separation between left and right Encoder.
    final static double lateralOffset = -6.0;  // offset between origin of robot and lateral Encoder.
    final static double R = 1.0;  // Encoder wheel radius.
    final static double encoderTicksPerRev = 1120;  // Ticks read per revolution of REV Encoder.
    final static double inchPerTick = 2.0 * Math.PI * R / encoderTicksPerRev;  // Inches traveled per tick moved.


    // Velocity

    int cycleToSkip = 20;

    public Robot(HardwareMap hardwareMap) {
        hwMap = hardwareMap;

        frontRight = hardwareMap.dcMotor.get("front_right_motor");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft = hardwareMap.dcMotor.get("front_left_motor");
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft = hardwareMap.dcMotor.get("back_left_motor");
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight = hardwareMap.dcMotor.get("back_right_motor");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lateralEncoder = hardwareMap.get(DcMotorEx.class, "enc_x");
        lateralEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        lateralEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lateralEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightEncoder = hardwareMap.get(DcMotorEx.class, "enc_right");
        //rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEncoder = hardwareMap.get(DcMotorEx.class, "enc_left");
        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopDrive();
        resetDriveEncoders();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        DriveTrain = new DriveTrain(this);
        IMU = new IMU(imu);

    }

    public HardwareMap getHardwareMap() {
        return hwMap;
    }

    public BNO055IMU getImu() {
        return imu;
    }

    public DcMotor getBackLeft() {
        return backLeft;
    }

    public DcMotor getFrontLeft() {
        return frontLeft;
    }

    public DcMotor getBackRight() {
        return backRight;
    }

    public DcMotor getFrontRight() {
        return frontRight;
    }

    public DcMotor getLeftEncoder() {
        return leftEncoder;
    }

    public DcMotor getRightEncoder() {
        return rightEncoder;
    }

    public DcMotor getFrontEncoder() {
        return lateralEncoder;
    }


    private void resetDriveEncoders() {
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lateralEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lateralEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // update variables
    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentLateralPosition = 0;

    private int oldRightPosition = 0;
    private int oldLeftPosition = 0;
    private int oldLateralPosition = 0;

    public Pose2D START_POSITION = new Pose2D(0, 0, AngleUtil.interpretAngle(90));  // Default

    public void setSTART_POSITION(Pose2D START) {
        START_POSITION = START;
        pos = START;
    }

    public Pose2D pos = START_POSITION;

    public void updateOdometry() { // make update() --> odometry() if need be [Merge with UpdateVelocities()
        currentRightPosition = rightEncoder.getCurrentPosition(); // Invert in Necessary
        currentLeftPosition = leftEncoder.getCurrentPosition(); // Invert in Necessary
        currentLateralPosition = lateralEncoder.getCurrentPosition(); // Invert in Necessary

        int dnRight = currentRightPosition - oldRightPosition;
        int dnLeft = currentLeftPosition - oldLeftPosition;
        int dnLateral = currentLateralPosition - oldLateralPosition;

        double dtheta = (dnLeft - dnRight) / L;
        double dx = (dnLeft+dnRight) / 2.0;
        double dy = dnLateral - lateralOffset * dtheta;

        dtheta *= inchPerTick;
        dx *= inchPerTick;
        dy *= inchPerTick;

        //double theta = pos.heading + (dtheta / 2.0);  // Does same thing as pos.heading | Might remove
        double dxTraveled = dx * Math.cos(pos.heading) - dy * Math.sin(pos.heading);
        double dyTraveled = dx * Math.sin(pos.heading) + dy * Math.cos(pos.heading);

        pos.x -= dxTraveled;  // Inverted cuz it was negative? :)
        pos.y += dyTraveled;
        pos.heading += dtheta;

        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldLateralPosition = currentLateralPosition;
    }

    double oldX = 0;
    double oldY = 0;
    double oldH = 0;
    double oldTime = 0;

    double currentX = 0;
    double currentY = 0;
    double currentH = 0;
    double currentTime = 0;

    public double dx = 0;
    public double dy = 0;
    public double dt = 0;
    public double dTheta = 0;
    int i = 0;

    public void updateVelocity() { // make update() --> odometry() if need be [Merge With UpdateOdometry()]
        i++; if(i % cycleToSkip == 0) {
                oldX = currentX;
                oldY = currentY;
                oldH = currentH;
                oldTime = currentTime;

                currentX = pos.x;
                currentY = pos.y;
                currentH = pos.getHeading();
                currentTime = System.nanoTime()/1e+9;

                dx = currentX - oldX;
                dy = currentY - oldY;
                dTheta = currentH - oldH;
                dt = currentTime - oldTime;

                pos.xVelocity = dx/dt;
                pos.yVelocity = dy/dt;
                pos.headingVelocity = dTheta/dt;

                pos.velocitySum = pos.xVelocity + pos.yVelocity + pos.headingVelocity;
        }
    }


    public void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

}