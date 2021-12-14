package org.firstinspires.ftc.teamcode.PurePursuit;

import org.firstinspires.ftc.teamcode.math.Point;

import java.util.ArrayList;

public class Path {


/**

    public static ArrayList<Waypoint> convertToPointArray(double[][] path) {
        ArrayList<Waypoint> newPath = new ArrayList<>();

        for(int i = 0; i < path.length; i++) {
            newPath.add(new Waypoint( new Point(path[i][0],
                    path[i][1]
            )));
        }

        return newPath;
    }

    public static double[][] clone2dDoubleArray(double[][] arr) {
        double[][] clonedArr = new double[arr.length][arr[0].length];
        for(int i = 0; i < arr.length; i++) {
            clonedArr[i] = arr[i];
        }
        return clonedArr;
    }

    public static double[][] convertToDoubleArray(ArrayList<Waypoint> path) {
        double[][] arrayPath = new double[path.size()][2];
        for(int i = 0; i < path.size(); i++) {
            arrayPath[i][0] = path.get(i).pos.x;
            arrayPath[i][1] = path.get(i).pos.y;
        }
        return arrayPath;
    }
 */

    /**

    public void interpolatePath(double spacing) {
        ArrayList<Waypoint> newPath = new ArrayList<>();
        int segments = path.size()-1;
        double initialRatio = 100 / ((spacing + 1) * 100);

        newPath.add(path.get(0));

        for(int i = 0; i <= segments-1; i++) {
            double ratio = 100 / ((spacing + 1) * 100);
            for(int j = 0; j < spacing + 1; j++) {                                                  // new point = current + ratio * (next - current)
                newPath.add(new Waypoint(path.get(i).pos.add(path.get(i+1).pos.subtract(path.get(i).pos).scalar(ratio))));    // current + (next - current) * ratio
                ratio += initialRatio;
            }
        }
    }

    public void smoothPath(double smoothing, double weight, double tolerance) {
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
     */
}
