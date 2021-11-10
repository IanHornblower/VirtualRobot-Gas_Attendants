package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Controller;
import static org.firstinspires.ftc.teamcode.util.Controller.*;
import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import java.util.concurrent.Future;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Autonomous(name = "Testing Auto", group = "Testing")
public class TestAuto extends LinearOpMode {

    private ExecutorService executor
            = Executors.newSingleThreadExecutor();

    public Future<Boolean> followPath(Trajectory path) {
        return executor.submit(() -> {
            path.followPath(Trajectory.PATH_TYPE.DIFFERENTIAL_PURE_PURSUIT, 5, 1);
            return true;
        });
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));


        Trajectory path1 = new Trajectory(robot, robot.START_POSITION);

        path1.addWaypoint(new Pose2D(0, 16, 0));
        path1.addWaypoint(new Pose2D(16, 32, 0));
        path1.forward(16, Robot.controlType.FIELD);

        waitForStart();

        CornettCore motionProfile = new CornettCore(robot);

        Future<Boolean> plan = this.followPath(path1);

        while(opModeIsActive()) {
            robot.updateOdometry();
            robot.updateVelocity();



            if(plan.isDone()) {
                telemetry.addData("Static XYH", new Pose2D(20, 60, 0).toString());
                telemetry.addData("XYH", robot.pos.toString());
                telemetry.update();
                stop();
            }

        }
    }
}
