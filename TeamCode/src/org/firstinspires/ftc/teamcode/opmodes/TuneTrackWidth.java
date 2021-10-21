package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

@Autonomous(name = "Calculate Track Width Error", group = "Tuning")
public class TuneTrackWidth extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(0)));

        waitForStart();

        while(opModeIsActive()) {
            robot.updateOdometry();
            robot.updateVelocity();

            CornettCore motionProfile = new CornettCore(robot);

            while(robot.IMU.getRawIMUHeadingInDegrees() < 3600) {
                robot.DriveTrain.setMotorPowers(1, -1);
            }
            robot.DriveTrain.stopDrive();
            
            telemetry.addData("XYH", robot.pos.toString());

            telemetry.addData("Direction", motionProfile.direction);
            telemetry.addData("PID Output", motionProfile.pidOutput);
            telemetry.addData("Output", motionProfile.output);
            telemetry.addData("Angle", Math.toDegrees(robot.pos.getHeading()));

            telemetry.update();

        }
    }
}