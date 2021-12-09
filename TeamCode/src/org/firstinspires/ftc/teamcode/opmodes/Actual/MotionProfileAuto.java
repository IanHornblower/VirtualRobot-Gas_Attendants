package org.firstinspires.ftc.teamcode.opmodes.Actual;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuitTrajectory;
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

@Autonomous(name = "Test Motion Profile", group = "Testing")
public class MotionProfileAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        // Paths

        Trajectory joe  = new Trajectory(robot, robot.START_POSITION);

        joe.addWaypoint(new Pose2D(0.4444444444444444, 0.2222222222222222, 0));
        joe.addWaypoint(new Pose2D(12.88888888888889,  11.11111111111111, 0));
        joe.addWaypoint(new Pose2D(31.11111111111111,  13.555555555555555, 0));
        joe.addWaypoint(new Pose2D(16.88888888888889,  34.44444444444444, 0));
        joe.addWaypoint(new Pose2D(24.22222222222222,  48.666666666666664, 0));

        waitForStart();

        CornettCore motionProfile = new CornettCore(robot);

        while(opModeIsActive()) {
            robot.updateOdometry();
            robot.updateVelocity();

            // Start

            joe.followPath(Trajectory.PATH_TYPE.DIFFERENTIAL_PURE_PURSUIT, 8, 1);


            // End

            stop();
        }
    }
}
