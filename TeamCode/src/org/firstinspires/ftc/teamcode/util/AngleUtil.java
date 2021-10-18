package org.firstinspires.ftc.teamcode.util;

public class AngleUtil {

    public static double fourPI = 4*Math.PI;

    /** if the length degrees is larger than 180 subtract it
     *  from 360 to get the shortest length from degrees to 0
     * @param degrees
     * @return
     */
    public static double normalizeLengthDegrees(double degrees) {
        if(degrees > 180) return 360-degrees;
        else return degrees;
    }

    public static double normalizeLengthRadians(double radians) {
        return Math.toRadians(normalizeLengthDegrees(Math.toDegrees(radians)));
    }

    /**
     * Keep input angle (radians) positive
     * @param radians
     * @return
     */
    public static double normalizeRadians(double radians) {
        while (radians<-Math.PI){
            radians += 2.0*Math.PI;
        }
        while (radians>Math.PI){
            radians -= 2.0*Math.PI;
        }
        return radians;
    }

    /**
     * Keep input angle (degrees) positive
     * @param angle
     * @return
     */
    public static double normalizeDegrees(double angle) {
        angle =  Math.toRadians(angle);
        while (angle<-Math.PI){
            angle += 2.0*Math.PI;
        }
        while (angle>Math.PI){
            angle -= 2.0*Math.PI;
        }
        return Math.toDegrees(angle);
    }

    public static double getCoterminalAngle(double degrees) {
        if(degrees > 360) {
            degrees -= 360;
        }
        if(degrees < -360) {
            degrees += 360;
        }
        return degrees;
    }

    public static double getCoterminalAngleInRadians(double radians) {
        if(radians > 2.0*Math.PI) {
            radians -= 2.0*Math.PI;
        }
        if(radians < -2.0*Math.PI) {
            radians += 2.0*Math.PI;
        }
        return radians;
    }

    public static double invertAngle(double angle) {
        return ((angle + Math.PI) % (2 * Math.PI));
    }

    public static double normalize(double angle) {
        angle = Math.toDegrees(angle);
        return Math.toRadians((angle >= 0 ? angle : (360 - ((-angle) % 360))) % 360);
    }

    public static double fixTheta(double theta) {
        theta *= -1;

        if(theta > Math.PI) {
            double dTheta = theta - Math.PI;
            theta = -Math.PI + dTheta;
        }
        if(theta < -Math.PI) {
            double dTheta = theta + Math.PI;
            theta = dTheta + Math.PI;
        }

        return theta-Math.PI;
    }

    public static double deNormalizeAngle(double angle) {
        if (angle > Math.PI) {
            angle -= Math.PI * 2.0;
        }
        return angle;
    }

    public static double avoidZeroError(double angle) {
        switch (Double.toString(angle)) {
            case "0.0":
                return 0;
            case "180.0":
                return 179.99;
            case "-180.0":
                return -179.99;
            default:
                return angle;
        }
    }

    public static double interpretAngle(double degrees) {
        return Math.toRadians(180 - degrees);
    }

    /**
     *
     * @param d Small
     * @param power
     * @return
     */
    public static double powRetainingSign(double d, double power) {
        // In case d is super small, just make it zero
        if (Math.abs(d) < 1e-14) {
            return 0;
        }
        return Math.copySign(Math.pow(Math.abs(d), power), d);
    }

}
