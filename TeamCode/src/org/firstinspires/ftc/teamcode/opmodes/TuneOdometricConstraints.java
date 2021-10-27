package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import static org.firstinspires.ftc.teamcode.hardware.Robot.*;

@Autonomous(name = "Calculate Track Width Error", group = "Tuning")
public class TuneOdometricConstraints extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(0)));

        boolean ran = false;

        telemetry.addLine("Press X to start");
        telemetry.update();

        do {
            if(gamepad1.x || gamepad2.x) {

                ran = true;
            }
        } while(!ran);

        telemetry.clear();

        waitForStart();

        while(opModeIsActive()) {
            robot.updateOdometry();
            robot.updateVelocity();
            robot.updateAcumulatedHeading();

            CornettCore motionProfile = new CornettCore(robot);

            robot.updateOdometry();
            robot.updateVelocity();
            robot.updateAcumulatedHeading();

            motionProfile.tuneTrackWidthIMU(Math.toRadians(720), 1);
            telemetry.addData("XYH", robot.pos.toString());
            telemetry.addData("Odom Angle", robot.accumulatedHeading);
            telemetry.addData("IMU Angle", robot.IMU.getAccumulatedHeadingInDegrees());
            telemetry.addData("Latteral Encoder Reading", robot.getFrontEncoder().getCurrentPosition());

            telemetry.addData("\nError", robot.IMU.getAccumulatedHeadingInDegrees()-robot.accumulatedHeading);
            telemetry.addData("Error Percent", robot.IMU.getAccumulatedHeadingInDegrees()/robot.accumulatedHeading);
            telemetry.addData("Track Width Multiplier", robot.accumulatedHeading/robot.IMU.getAccumulatedHeadingInDegrees());
            telemetry.addData("New Track Width (L) ", robot.L * (robot.accumulatedHeading/robot.IMU.getAccumulatedHeadingInDegrees()));

            telemetry.addData("\nLatteral Encoder Offset", (robot.getFrontEncoder().getCurrentPosition()/Math.toRadians(robot.IMU.getAccumulatedHeadingInDegrees()))*inchPerTick);
            telemetry.update();
        }
    }
}