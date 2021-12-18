package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.math.Point;

import java.util.ArrayList;

public class Path {

    private ArrayList<Point> path;
    private double smoothing = 1;
    private double weight = 1;
    private double tolerance = 1;

    public Path(ArrayList<Point> path) {
        this.path = path;
    }

    public Path(ArrayList<Point> path, double smoothing, double weight, double tolerance) {
        this.path = path;
        this.smoothing = smoothing;
        this.weight = weight;
        this.tolerance = tolerance;
    }

    public ArrayList<Point> getPath() {
        return path;
    }

    public void setPath(ArrayList<Point> path) {
        this.path = path;
    }

    public void setSmoothing(double smoothing) {
        this.smoothing = smoothing;
    }

    public void setWeight(double weight) {
        this.weight = weight;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public static ArrayList<Point> convertToPointArray(double[][] path) {
        ArrayList<Point> newPath = new ArrayList<>();

        for (double[] doubles : path) {
            newPath.add(new Point(doubles[0],
                    doubles[1]
            ));
        }

        return newPath;
    }

    public static double[][] clone2dDoubleArray(double[][] arr) {
        double[][] clonedArr = new double[arr.length][arr[0].length];
        System.arraycopy(arr, 0, clonedArr, 0, arr.length);
        return clonedArr;
    }

    public static double[][] convertToDoubleArray(ArrayList<Point> path) {
        double[][] arrayPath = new double[path.size()][2];
        for(int i = 0; i < path.size(); i++) {
            arrayPath[i][0] = path.get(i).x;
            arrayPath[i][1] = path.get(i).y;
        }
        return arrayPath;
    }

    public void interpolatePath(double spacing) {
        ArrayList<Point> newPath = new ArrayList<>();
        int segments = path.size()-1;
        double initialRatio = 100 / ((spacing + 1) * 100);

        newPath.add(path.get(0));

        for(int i = 0; i <= segments-1; i++) {
            double ratio = 100 / ((spacing + 1) * 100);
            for(int j = 0; j < spacing + 1; j++) {                                                  // new point = current + ratio * (next - current)
                newPath.add(path.get(i).add(path.get(i+1).subtract(path.get(i)).scalar(ratio)));    // current + (next - current) * ratio
                ratio += initialRatio;
            }
        }
        path = newPath;
    }

    public void smoothPath() {
        double[][] arrayPath = convertToDoubleArray(path);

        double[][] newPath = clone2dDoubleArray(arrayPath);
        double change = tolerance;

        while(change >= tolerance) {
            change = 0.0;
            for(int i = 1; i < arrayPath.length - 1; i++) {
                for(int j = 0; j < arrayPath[i].length; j++) {
                    double aux = newPath[i][j];

                    newPath[i][j] += smoothing*(arrayPath[i][j]-newPath[i][j]) + weight*(newPath[i-1][j]+newPath[i+1][j]-2*newPath[i][j]);

                    //newPath[i][j] += smoothing * (arrayPath[i][j] - newPath[i][j]) + weight * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
            }
        }

        path = convertToPointArray(newPath);
    }
}
