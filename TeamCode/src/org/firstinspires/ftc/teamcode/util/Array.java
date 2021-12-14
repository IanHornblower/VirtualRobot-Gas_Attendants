package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;

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


    public static ArrayList<Point> reversePointArray(ArrayList<Point> alist) {
        // Arraylist for storing reversed elements
        ArrayList<Point> revArrayList = new ArrayList<Point>();
        for (int i = alist.size() - 1; i >= 0; i--) {

            // Append the elements in reverse order
            revArrayList.add(alist.get(i));
        }

        // Return the reversed arraylist
        return revArrayList;
    }

    public static ArrayList<Pose2D> reversePose2DArray(ArrayList<Pose2D> alist) {
        // Arraylist for storing reversed elements
        ArrayList<Pose2D> revArrayList = new ArrayList<Pose2D>();
        for (int i = alist.size() - 1; i >= 0; i--) {

            // Append the elements in reverse order
            revArrayList.add(alist.get(i));
        }

        // Return the reversed arraylist
        return revArrayList;
    }
}