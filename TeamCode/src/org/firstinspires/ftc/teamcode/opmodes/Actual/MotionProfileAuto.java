package org.firstinspires.ftc.teamcode.opmodes.Actual;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.control.*;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.util.Time.await;
import static org.firstinspires.ftc.teamcode.util.Time.timeout;

@Autonomous(name = "Test Motion Profile", group = "Testing")
public class MotionProfileAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        waitForStart();

        CornettCore motionProfile = new CornettCore(robot);

        while(opModeIsActive()) {
            robot.updateOdometry();

            timeout(1500, ()-> robot.DriveTrain.setMotorPowers(-1, -1));
            robot.stopDrive();


            sleep(100000);
        }
    }
}
