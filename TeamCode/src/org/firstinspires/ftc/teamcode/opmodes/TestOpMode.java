package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Controller;
import static org.firstinspires.ftc.teamcode.util.Controller.*;
import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;

import org.firstinspires.ftc.teamcode.util.MathUtil;

@TeleOp(name = "Testing OpMode", group = "Testing")
public class TestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        waitForStart();

        while(opModeIsActive()) {
            robot.updateOdometry();
            //robot.updateVelocity();

            double leftX = AngleUtil.powRetainingSign(Controller.deadZone(gamepad1.left_stick_x, 0.1), LEFT_TRIGGER_X_POW);
            double leftY = AngleUtil.powRetainingSign(Controller.deadZone(-gamepad1.left_stick_y, 0.1), LEFT_TRIGGER_Y_POW);
            double turn = Controller.deadZone(gamepad1.right_stick_x, 0.05);

            robot.DriveTrain.driveFieldCentric(leftX, leftY, turn);

            telemetry.addData("XYH",
                    roundPlaces(robot.pos.x, 1) +
                    " " + roundPlaces(robot.pos.y, 1) +
                    " " + roundPlaces(robot.pos.getHeadingInDegrees(), 1)
            );

            telemetry.addData("XYT",
                    roundPlaces(leftX, 1) +
                     " " + roundPlaces(leftY, 1) +
                     " " + roundPlaces(turn, 1)
            );

            telemetry.update();

        }
    }
}
