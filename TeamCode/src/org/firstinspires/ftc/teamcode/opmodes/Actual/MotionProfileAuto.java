package org.firstinspires.ftc.teamcode.opmodes.Actual;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.control.Path;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

@Autonomous(name = "Test Motion Profile", group = "Testing")
public class MotionProfileAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        // Paths

        Trajectory joe  = new Trajectory(robot, robot.START_POSITION);

        joe.addWaypoint(new Point(-0.6666666666666666, 16.444444444444443));
        joe.addWaypoint(new Point(-16.88888888888889, 40.44444444444444));
        joe.addWaypoint(new Point(-13.333333333333334, 53.333333333333336));
        joe.addWaypoint(new Point(11.555555555555555, 56.0));
        joe.addWaypoint(new Point(20.0, 48.666666666666664));
        joe.addWaypoint(new Point(30.88888888888889, 12.222222222222221));
        joe.addWaypoint(new Point(56.22222222222222, 0.0));

        Trajectory oeJ = new Trajectory(robot, joe.end());

        oeJ.addWaypoint(new Point(56.22222222222222, 0.0));
        oeJ.addWaypoint(new Point(30.88888888888889, 12.222222222222221));
        oeJ.addWaypoint(new Point(20.0, 48.666666666666664));
        oeJ.addWaypoint(new Point(11.555555555555555, 56.0));
        oeJ.addWaypoint(new Point(-13.333333333333334, 53.333333333333336));
        oeJ.addWaypoint(new Point(-16.88888888888889, 40.44444444444444));
        oeJ.addWaypoint(new Point(-0.6666666666666666, 16.444444444444443));
        oeJ.addWaypoint(robot.START_POSITION);

        Path newJoe = new Path(joe.getPointArray(), 0.98, 0.02, 0.132);

        newJoe.interpolatePath(20);
        newJoe.smoothPath();

        joe = new Trajectory(robot, newJoe.getPath(), AngleUtil.interpretAngle(90));

        waitForStart();

        CornettCore motionProfile = new CornettCore(robot);

        while(opModeIsActive()) {

            joe.followPath(Trajectory.PATH_TYPE.BASIC, 1);



            sleep(100000);
        }
    }
}
