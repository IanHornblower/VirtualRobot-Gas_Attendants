package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.math.Pose2D;

public class Array {
    /**
     * Add to the end of regular double array
     * @param arr
     * @param x
     * @return
     */

    public static double[] appendArray(double arr[], double x) {
        int n = arr.length;
        double newarr[] = new double[n + 1];

        for (int i = 0; i < n; i++) {
            newarr[i] = arr[i];
        }
        newarr[n] = x;

        return newarr;
    }

    /**
     * Add to the end of regular Pose2D array
     * @param arr
     * @param pose
     * @return
     */

    public static Pose2D[] appendArray(Pose2D arr[], Pose2D pose) {
        int n = arr.length;
        Pose2D newarr[] = new Pose2D[n + 1];

        for (int i = 0; i < n; i++) {
            newarr[i] = arr[i];
        }
        newarr[n] = pose;

        return newarr;
    }
}