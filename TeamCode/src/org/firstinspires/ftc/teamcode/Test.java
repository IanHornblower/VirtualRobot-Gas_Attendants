package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.util.Init;
import org.firstinspires.ftc.teamcode.util.Time;

import javax.sound.midi.SysexMessage;
import java.util.concurrent.atomic.AtomicReference;

import static java.lang.Thread.sleep;

import static org.firstinspires.ftc.teamcode.util.Time.*;
import static org.firstinspires.ftc.teamcode.util.Init.*;

public class Test {
    public static void main(String[] args) {
        asyncTimeout(2000, ()-> System.out.println("Going"));
    }
}
