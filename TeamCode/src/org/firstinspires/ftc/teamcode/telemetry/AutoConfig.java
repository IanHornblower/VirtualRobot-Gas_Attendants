package org.firstinspires.ftc.teamcode.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoConfig {

    Telemetry telemetry;
    String[][] config;

    public AutoConfig(Telemetry telemetry, String[][] config) {
        this.telemetry = telemetry;
        this.config = config;

    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setConfig(String[][] config) {
        this.config = config;
    }

    public void update() {
        int optionSize = config.length;

        String[] titles = new String[optionSize];
        for(int i = 0; i < optionSize; i++) {
            titles[i] = config[i][0];
        }



        telemetry.addLine();
    }
}
