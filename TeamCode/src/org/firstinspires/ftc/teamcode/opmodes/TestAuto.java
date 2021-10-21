package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Controller;
import static org.firstinspires.ftc.teamcode.util.Controller.*;
import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;

import org.firstinspires.ftc.teamcode.util.MathUtil;

@Autonomous(name = "Testing Auto", group = "Testing")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        waitForStart();

        while(opModeIsActive()) {
            robot.updateOdometry();
            robot.updateVelocity();

            CornettCore motionProfile = new CornettCore(robot);

            motionProfile.runToPositionSync(20, 20, Math.toRadians(90), 0.2);

            motionProfile.runToPositionSync(-20, 20, Math.toRadians(0), 0.2);
            
            telemetry.addData("XYH", robot.pos.toString());

            telemetry.addData("Direction", motionProfile.direction);
            telemetry.addData("PID Output", motionProfile.pidOutput);
            telemetry.addData("Output", motionProfile.output);
            telemetry.addData("Angle", Math.toDegrees(robot.pos.getHeading()));
            telemetry.addData("Is Running", robot.DriveTrain.isPIDRunning());

            telemetry.update();

        }
    }
}
