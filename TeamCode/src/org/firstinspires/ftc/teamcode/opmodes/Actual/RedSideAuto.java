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

@Autonomous(name = "Redauto", group = "Testing")
public class RedSideAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(63, 12, AngleUtil.interpretAngle(0)));

        // Paths

        Trajectory toTeamShippingHub = new Trajectory(robot, robot.START_POSITION);

        toTeamShippingHub.addWaypoint(new Pose2D(40, -2, AngleUtil.interpretAngle(30)));

        Trajectory joe  = new Trajectory(robot, toTeamShippingHub.end());

        joe.addWaypoint(new Pose2D(42.888888888888886, -0.8888888888888888, 0));
        joe.addWaypoint(new Pose2D(61.55555555555556, 20.22222222222222, 0));
        joe.addWaypoint(new Pose2D(62.44444444444444, 31.555555555555557, 0));
        joe.addWaypoint(new Pose2D(54.0, 37.111111111111114, 0));
        joe.addWaypoint(new Pose2D(40.22222222222222, 37.333333333333336, 0));
        joe.addWaypoint(new Pose2D(39.77777777777778, 56.666666666666664, 0));




        waitForStart();

        CornettCore motionProfile = new CornettCore(robot);

        while(opModeIsActive()) {
            robot.updateOdometry();
            robot.updateVelocity();

            // Start

            toTeamShippingHub.followPath(Trajectory.PATH_TYPE.BASIC, 1);

            //sleep(1000);

            joe.followPath(Trajectory.PATH_TYPE.DIFFERENTIAL_PURE_PURSUIT, 8, 1);


            // End

            stop();
        }
    }
}
