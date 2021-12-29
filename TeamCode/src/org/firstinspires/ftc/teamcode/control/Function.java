package org.firstinspires.ftc.teamcode.control;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class Function {

    Executor x;
    double distance;

    public Function(double distance, Executor x) {
        this.distance = distance;
        this.x = x;
    }

    public void execute() {
        x.method();
    }

}
