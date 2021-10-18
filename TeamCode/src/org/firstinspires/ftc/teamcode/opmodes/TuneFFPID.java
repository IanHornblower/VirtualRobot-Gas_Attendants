package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Controller;
import static org.firstinspires.ftc.teamcode.util.Controller.*;
import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;

@Autonomous(name = "FF PID", group = "Tuning")
public class TuneFFPID extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        waitForStart();

        while(opModeIsActive()) {
            robot.updateOdometry();
           //robot.updateVelocity();

            telemetry.addData("XYH",
                    roundPlaces(robot.pos.x, 1) +
                            " " + roundPlaces(robot.pos.y, 1) +
                            " " + roundPlaces(robot.pos.getHeadingInDegrees(), 1)
            );

            telemetry.update();

        }
    }
}
