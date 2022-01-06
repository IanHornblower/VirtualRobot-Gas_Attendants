package org.firstinspires.ftc.teamcode.opmodes.Actual;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.control.*;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

import java.util.ArrayList;

@Autonomous(name = "Test Motion Profile", group = "Testing")
public class MotionProfileAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        // Paths

        Trajectory joe  = new Trajectory(robot, robot.START_POSITION);

        joe.addWaypoint(new Point(0, 24));
        joe.addWaypoint(new Point(24, 36));
        joe.addWaypoint(new Point(24, 48));

        waitForStart();

        CornettCore motionProfile = new CornettCore(robot);

        while(opModeIsActive()) {
            robot.updateOdometry();

            ArrayList<Function> list = new ArrayList<>();

            list.add(new Function(12, () -> {
                telemetry.addLine("AT 12: \t" + robot.accumulatedDistance);
                telemetry.update();
            }));

            list.add(new Function(24,() -> {
                telemetry.addLine("do");
                telemetry.update();
            }));

            list.add(new Function(36, () -> {
                telemetry.addLine("AT 36: \t" + robot.accumulatedDistance);
                telemetry.update();
            }));


            joe.at(list).followPath(Trajectory.PATH_TYPE.PURE_PURSUIT, CornettCore.DIRECTION.FORWARD, 8, 1);

            /*

            ArrayList<Function> toExecute = new ArrayList<>();

            toExecute.add(new Function(5, () -> {
                System.out.println("2");
            }));


            joe.at(toExecute);

             */





            sleep(100000);
        }
    }
}
