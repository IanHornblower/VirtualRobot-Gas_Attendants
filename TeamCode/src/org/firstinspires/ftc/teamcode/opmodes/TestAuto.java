package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.telemetry.AutoConfig;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

import java.util.concurrent.Future;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Autonomous(name = "Testing Auto", group = "Testing")
public class TestAuto extends LinearOpMode {


    private ExecutorService executor
            = Executors.newSingleThreadExecutor();

    public Future<Boolean> followPath(Trajectory path) {
        return executor.submit(() -> {
            path.followPath(Trajectory.PATH_TYPE.BASIC, 1);
            return true;
        });
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));


        Trajectory path1 = new Trajectory(robot, robot.START_POSITION);

        path1.addWaypoint(new Pose2D(25, 25, 0));
        path1.addWaypoint(new Pose2D(60, 30, 0));
        path1.addWaypoint(new Pose2D(30, 73, 0));

        String[][] config = {
                {"Side", "Red", "Blue"},
                {"Parking", "Warehouse", "Storage"},
        };

        waitForStart();

        CornettCore motionProfile = new CornettCore(robot);

        //Future<Boolean> plan = this.followPath(path1);

        //PurePursuitTrajectory ppTraj = new PurePursuitTrajectory(robot, robot.START_POSITION, 20, 2);

        //ppTraj.addWaypoint(new Waypoint(new Point(12, 12)));
        //ppTraj.addWaypoint(new Waypoint(new Point(12, 24)));

        //ppTraj.buildTrajectory();

        while(opModeIsActive()) {

            motionProfile.runToPosition(12, 12, Math.toRadians(90));


        }
    }
}
