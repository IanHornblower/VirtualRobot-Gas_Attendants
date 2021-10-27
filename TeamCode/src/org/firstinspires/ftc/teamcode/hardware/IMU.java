package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.math.Pose2D;

public class IMU {

    BNO055IMU imu;
    Orientation angles;

    double previousHeading = 0;
    double accumulatedHeading = 0;

    public IMU() {

    }

    public IMU(BNO055IMU imu) {
        this.imu = imu;
    }

    public void initialize(){
        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu.initialize(IMUParameters);
    }

    // Remove Comments when using in IRL SDK

    /**
    public boolean getisGyroCalibrated(){
        return imu.isGyroCalibrated();
    }

    public BNO055IMU.CalibrationStatus getCalibrationStatus(){
        return imu.getCalibrationStatus();
    }
    */

    public double getRawIMUHeadingInDegrees() {
        double currentHeading;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = -angles.firstAngle;
        return currentHeading;
    }
    public double getIMUHeadingInDegrees() {
        double currentHeading;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = -angles.firstAngle;
        if (currentHeading < 0) {
            currentHeading = -currentHeading;
        } else {
            currentHeading = 360 - currentHeading;
        }
        return Math.abs(currentHeading);
    }

    public double getAccumulatedHeadingInDegrees() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = -angles.firstAngle;
        double dHeading = currentHeading - previousHeading;

        if(dHeading < -180) {
            dHeading += 360;
        }
        else if(dHeading >= 180) {
            dHeading -=360;
        }

        accumulatedHeading += dHeading;
        previousHeading = currentHeading;

        return accumulatedHeading;
     }

    public double getIMUHeading() {
        return Math.toRadians(getIMUHeadingInDegrees());
    }

    public double getRawIMUHeading() {
        return Math.toRadians(getRawIMUHeadingInDegrees());
    }

    public double[] printAngles(){
        double[] values;

        values = new double[3];
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        values[0] = angles.firstAngle;
        values[1] = angles.secondAngle;
        values[2] = angles.thirdAngle;

        return values;
    }

    public void resetAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angles.firstAngle = 0;
    }

    public double headingAdjustment(double targetHeading){
        double adjustment;
        double currentHeading;
        double degreesOff;
        boolean goRight;

        currentHeading = getIMUHeading();

        goRight = targetHeading > currentHeading;
        degreesOff = Math.abs(targetHeading - currentHeading);

        if (degreesOff > 180) {
            goRight = !goRight;
            degreesOff = 360 - degreesOff;
        }

        if (degreesOff < .3) {
            adjustment = 0;
        } else {
            adjustment = (Math.pow((degreesOff + 2) / 5, 2) + 2) / 100;
        }

        if (goRight) {
            adjustment = -adjustment;
        }
        return adjustment;
    }
}