package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import javafx.geometry.Pos;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

public class Robot extends OpMode {
    private DcMotor backLeft, backRight, frontLeft, frontRight;

    private DcMotorEx leftEncoder, rightEncoder, lateralEncoder;

    private BNO055IMU imu;
    private Orientation angles;

    private double previousHeading;
    private Pose2D previousPosition = new Pose2D(0, 0, 0);

    public HardwareMap hwMap;
    public DriveTrain DriveTrain;
    public IMU IMU;

    public enum controlType{ROBOT, FIELD}

    // Robot Kinematics

    // Odmometric Constraints
    public final static double L = 12;  // separation between left and right Encoder.
    public final static double lateralOffset = -6;  // offset between origin of robot and lateral Encoder.
    public final static double R = 1.0;  // Encoder wheel radius.
    public final static double encoderTicksPerRev = 1120;  // Ticks read per revolution of REV Encoder.
    public final static double inchPerTick = 2.0 * Math.PI * R / encoderTicksPerRev;  // Inches traveled per tick moved.

    public final static double TRACKWIDTH = 15;


    // Velocity

    int cycleToSkip = 1;

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

    public double accumulatedHeading = 0;
    public double accumulatedDistance = 0;

    public double leftVelocity = 0;
    public double rightVelocity = 0;
    public double lateralVelocity = 0;

    public double dxTraveled = 0;
    public double dyTraveled = 0;

    public Pose2D START_POSITION = new Pose2D(0, 0, AngleUtil.interpretAngle(90));  // Default

    public void setSTART_POSITION(Pose2D START) {
        START_POSITION = START;
        pos = START;
    }

    public Pose2D pos = START_POSITION;

    int i = 0;

    public void pass() {
        System.out.print(accumulatedDistance);
    }


    public void updateAcumulatedHeading() {
        double currentHeading = Math.toDegrees(pos.getHeading());

        double dHeading = currentHeading - previousHeading;

        if(dHeading < -180) {
            dHeading += 360;
        }
        else if(dHeading >= 180) {
            dHeading -=360;
        }

        accumulatedHeading -= dHeading;
        previousHeading = currentHeading;
    }

    public void updateOdometry() { // make update() --> odometry() if need be [Merge with UpdateVelocities()
        currentRightPosition = rightEncoder.getCurrentPosition(); // Invert in Necessary
        currentLeftPosition = leftEncoder.getCurrentPosition(); // Invert in Necessary
        currentLateralPosition = lateralEncoder.getCurrentPosition(); // Invert in Necessary

        int dnRight = currentRightPosition - oldRightPosition;
        int dnLeft = currentLeftPosition - oldLeftPosition;
        int dnLateral = currentLateralPosition - oldLateralPosition;

        double dtheta = (dnLeft - dnRight) / L;
        double dx = (dnLeft + dnRight) / 2.0;
        double dy = dnLateral - lateralOffset * dtheta;

        dtheta *= inchPerTick;
        dx *= inchPerTick;
        dy *= inchPerTick;

        //double theta = pos.heading + (dtheta / 2.0);  // Does same thing as pos.heading | Might remove
        double dxTraveled = dx * Math.cos(pos.heading) - dy * Math.sin(pos.heading);
        double dyTraveled = dx * Math.sin(pos.heading) + dy * Math.cos(pos.heading);

        accumulatedDistance += Math.hypot(dxTraveled, dyTraveled);

        pos.x -= dxTraveled;  // Inverted cuz it was negative? :)
        pos.y += dyTraveled;
        pos.heading += dtheta;

        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldLateralPosition = currentLateralPosition;
    }

    public void updateEncoderVelocity() {
        leftVelocity = leftEncoder.getVelocity()*inchPerTick;
        rightVelocity = rightEncoder.getVelocity()*inchPerTick;
        lateralVelocity = lateralEncoder.getVelocity()*inchPerTick;
    }

    public void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}