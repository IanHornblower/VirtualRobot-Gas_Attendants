package org.firstinspires.ftc.teamcode.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Array;

import javax.sound.midi.SysexMessage;
import java.util.ArrayList;
import java.util.Arrays;

public class AutoConfig {

    Telemetry telemetry;

    int row = 0;
    int stringLengths = 0;

    ArrayList<String[]> format = new ArrayList<>();

    ArrayList<String[]> config = new ArrayList<>();

    public AutoConfig(Telemetry telemetry, ArrayList<String[]> config) {
        this.telemetry = telemetry;
        this.config = config;
    }

    public void parseConfig() {
        // Get largest amount of Rows
        for (String[] strings : config) {
            row = Math.max(row, strings.length);
        }

        stringLengths = (row*4) - 1;

        for (int i = 0; i < config.size(); i++) {
            format.add(new String[stringLengths]);
        }

        for (int i = 0; i < format.size(); i++) {
            for (int j = 0; i < format.get(i).length-2; i++) {
                switch (j) {
                    case 0:
                        format.get(i)[j] = ">";
                        break;
                    case 1:
                        format.get(i)[j] = config.get(i)[j];
                    case 2:
                        format.get(i)[j] = "<";
                    default:
                        format.get(i)[j] = "\t";
                }
            }
        }
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setConfig(ArrayList<String[]> config) {
        this.config = config;
    }

    public void update() {
        telemetry.addData("Row", row);
        for(int i = 0; i < row; i++) {
            telemetry.addLine("Line: " + (i+1));
        }
        telemetry.update();

        for (String[] strings : format) {
            System.out.println(Arrays.toString(strings));
        }
    }

    public void it() {
        ArrayList<String[]> joe = new ArrayList<>();

        joe.add(new String[] {"BLUE", "RED"});
        joe.add(new String[] {"WAREHOUSE", "STORAGE"});
        joe.add(new String[] {"DUCK, CYCLE, NEITHER"});

    }
}
