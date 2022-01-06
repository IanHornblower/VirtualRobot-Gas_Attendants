package org.firstinspires.ftc.teamcode.opmodes.Actual;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Controller;

import java.util.concurrent.atomic.AtomicReference;

import static org.firstinspires.ftc.teamcode.util.Controller.LEFT_TRIGGER_X_POW;
import static org.firstinspires.ftc.teamcode.util.Controller.LEFT_TRIGGER_Y_POW;

@Autonomous(name = "Telemetry GUI", group = "Testing")
public class TelemetryGUI extends LinearOpMode {
    public static enum LIFTSTATE {
        ONE,
        TWO,
        THREE;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        final LIFTSTATE[] lift = {LIFTSTATE.ONE};

        waitForStart();

        CornettCore motionProfile = new CornettCore(robot);

        while(opModeIsActive()) {
            Thread t1 = new Thread(() -> {
                switch (lift[0]) {
                    case ONE:
                        sleep(200);
                        if(gamepad2.dpad_right) lift[0] = LIFTSTATE.TWO;
                        if(gamepad2.dpad_left) lift[0] = LIFTSTATE.THREE;
                        break;
                    case TWO:
                        sleep(200);
                        if(gamepad2.dpad_right) lift[0] = LIFTSTATE.THREE;
                        if(gamepad2.dpad_left) lift[0]  = LIFTSTATE.ONE;
                        break;
                    case THREE:
                        sleep(200);
                        if(gamepad2.dpad_right)  lift[0] = LIFTSTATE.ONE;
                        if(gamepad2.dpad_left) lift[0] = LIFTSTATE.TWO;
                        break;
                }
            });

            t1.start();

            robot.updateOdometry();

            double leftX = AngleUtil.powRetainingSign(Controller.deadZone(gamepad2.left_stick_x, 0.1), LEFT_TRIGGER_X_POW);
            double leftY = AngleUtil.powRetainingSign(Controller.deadZone(-gamepad2.left_stick_y, 0.1), LEFT_TRIGGER_Y_POW);
            double turn = Controller.deadZone(gamepad2.right_stick_x, 0.1);

            robot.DriveTrain.driveFieldCentric(leftX, leftY, turn);

            if(gamepad2.a) {
                switch (lift[0]) {
                    case ONE:
                        System.out.println(lift[0].toString());
                    case TWO:
                        System.out.println(lift[0].toString());
                    case THREE:
                        System.out.println(lift[0].toString());
                }
            }

            telemetry.addData("Lift Level", lift[0].toString());
            telemetry.update();

            //stop();
        }
    }
}
