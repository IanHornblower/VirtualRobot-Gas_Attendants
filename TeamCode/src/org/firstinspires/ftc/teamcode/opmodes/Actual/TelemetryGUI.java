package org.firstinspires.ftc.teamcode.opmodes.Actual;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.control.Path;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.telemetry.AutoConfig;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name = "Telemetry GUI", group = "Testing")
public class TelemetryGUI extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        CornettCore motionProfile = new CornettCore(robot);

        while(opModeIsActive()) {
            // Start

            ArrayList<String[]> joe = new ArrayList<>();

            joe.add(new String[] {"BLUE", "RED"});
            joe.add(new String[] {"WAREHOUSE", "STORAGE"});
            joe.add(new String[] {"DUCK", "CYCLE", "NEITHER"});

            AutoConfig config = new AutoConfig(telemetry, joe);

            config.parseConfig();

            config.update();


            sleep(10000);

            stop();
        }
    }
}
