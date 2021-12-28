package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Controller;
import static org.firstinspires.ftc.teamcode.util.Controller.*;

@Disabled
@TeleOp(name = "Testing OpMode", group = "Testing")
public class TestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        double x = 0;
        double y = 0;
        double h = 0;

        waitForStart();

        while(opModeIsActive()) {
            robot.updateOdometry();
            robot.updateVelocity();
            robot.updateEncoderVelocity();
            robot.updateAcumulatedHeading();

            double leftX = AngleUtil.powRetainingSign(Controller.deadZone(gamepad1.left_stick_x, 0.1), LEFT_TRIGGER_X_POW);
            double leftY = AngleUtil.powRetainingSign(Controller.deadZone(-gamepad1.left_stick_y, 0.1), LEFT_TRIGGER_Y_POW);
            double turn = Controller.deadZone(gamepad1.right_stick_x, 0.1);

            robot.DriveTrain.driveFieldCentric(leftX, leftY, turn);

            telemetry.addData("XYH", robot.pos.toString());
            telemetry.addData("left Velo", robot.leftVelocity);
            telemetry.addData("right Velo", robot.rightVelocity);
            telemetry.update();
        }
    }
}
