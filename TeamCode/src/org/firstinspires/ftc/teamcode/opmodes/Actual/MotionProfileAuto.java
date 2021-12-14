package org.firstinspires.ftc.teamcode.opmodes.Actual;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
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

        //joe.addWaypoint(new Pose2D(0.0, 0.4444444444444444, 0));
        joe.addWaypoint(new Pose2D(29.77777777777778, 24.88888888888889, 0));
        joe.addWaypoint(new Pose2D(5.555555555555555, 41.77777777777778, 0));
        joe.addWaypoint(new Pose2D(-29.77777777777778, 13.555555555555555, 0));
        joe.addWaypoint(new Pose2D(-50.888888888888886, 45.111111111111114, 0));


        //joe.addWaypoint(new Pose2D(-0.2222222222222222, 0.6666666666666666, 0));
        //joe.addWaypoint(new Pose2D(2.0, 38.0, 0));
        //joe.addWaypoint(new Pose2D(22.0, 49.55555555555556, 0));
        //joe.addWaypoint(new Pose2D(38.22222222222222, 49.77777777777778, 0));
        //joe.addWaypoint(new Pose2D(50.0, 29.11111111111111, 0));
        //joe.addWaypoint(new Pose2D(52.888888888888886, 8.444444444444445, 0));
        //joe.addWaypoint(new Pose2D(50.888888888888886, -17.555555555555557, 0));
        //joe.addWaypoint(new Pose2D(30.88888888888889, -42.888888888888886, 0));
        //joe.addWaypoint(new Pose2D(-11.11111111111111, -40.44444444444444, 0));
        //joe.addWaypoint(new Pose2D(-28.0, -12.222222222222221, 0));
        //joe.addWaypoint(new Pose2D(-26.88888888888889, 16.444444444444443, 0));
        //joe.addWaypoint(new Pose2D(-10.666666666666666, 43.55555555555556, 0));
        //joe.addWaypoint(new Pose2D(-21.333333333333332, 58.44444444444444, 0));
        //joe.addWaypoint(new Pose2D(-42.666666666666664, 54.22222222222223, 0));





        //Trajectory backJoe = new Trajectory(robot, joe.end());
        //backJoe.addWaypoint(new Pose2D(25.333333333333332, 48.666666666666664, 0));
        //backJoe.addWaypoint(new Pose2D(34.666666666666664, 26.88888888888889, 0));
        //backJoe.addWaypoint(new Pose2D(23.77777777777778, 10.0, 0));
        //backJoe.addWaypoint(new Pose2D(-0.6666666666666666, 0.2222222222222222, 0));

        Trajectory backJoe = joe.retrace();

        waitForStart();

        CornettCore motionProfile = new CornettCore(robot);

        while(opModeIsActive()) {
            robot.updateOdometry();
            robot.updateVelocity();

            // Start

            joe.followPath(Trajectory.PATH_TYPE.PURE_PURSUIT, CornettCore.DIRECTION.FORWARD, 8, 1);
            //robot.DriveTrain.stopDrive();

            sleep(100000);

            //backJoe.followPath(Trajectory.PATH_TYPE.PURE_PURSUIT, CornettCore.DIRECTION.BACKWARD, 8, 1);

            // End

            //stop();
        }
    }
}
