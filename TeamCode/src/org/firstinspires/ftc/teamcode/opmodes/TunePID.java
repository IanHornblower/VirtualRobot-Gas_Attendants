package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Controller;
import static org.firstinspires.ftc.teamcode.util.Controller.*;
import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;

@TeleOp(name = "TunePID", group = "Tuning")
public class TunePID extends LinearOpMode {

    public enum type{ROTATIONAL, DIRECTIONAL}

    type PIDSelector;

    public int ROTATIONAL = 0;
    public int DIRECTIONAL = 1;

    double scaleBox = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        boolean ran = false;
        telemetry.addLine("Press X for tuning heading PID\tPress B for Tuning xy PID");
        telemetry.update();

        do {
            if (gamepad1.x || gamepad2.x) {
                ran = true;
                PIDSelector = type.ROTATIONAL;
            }

            if (gamepad1.b || gamepad2.b) {
                ran = true;
                PIDSelector = type.DIRECTIONAL;
            }

        } while(!ran);

        telemetry.clear();

        telemetry.addData("Tuning", PIDSelector);
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            CornettCore motionProfile = new CornettCore(robot);

            switch (PIDSelector) {
                case ROTATIONAL:
                    robot.updateOdometry();

                    motionProfile.rotateSync(Math.toRadians(0), Math.toRadians(0.2));

                    motionProfile.rotateSync(Math.toRadians(90), Math.toRadians(0.2));

                    telemetry.addData("XYH", robot.pos.toString());
                    telemetry.update();
                    break;
                case DIRECTIONAL:
                    robot.updateOdometry();

                    motionProfile.runToPositionSync(10*scaleBox, 0, Math.toRadians(90), 0.5);

                    motionProfile.runToPositionSync(10*scaleBox, 10*scaleBox, Math.toRadians(90), 0.5);

                    motionProfile.runToPositionSync(0, 10*scaleBox, Math.toRadians(90), 0.5);

                    motionProfile.runToPositionSync(0, 0, Math.toRadians(90), 0.5);

                    telemetry.addData("XYH", robot.pos.toString());
                    telemetry.update();
                    break;
                default:
                    telemetry.addLine("Well something is fucked?");
                    telemetry.addLine("Prolly ur enums dumbfuck");
                    telemetry.update();
            }
        }
    }
}
