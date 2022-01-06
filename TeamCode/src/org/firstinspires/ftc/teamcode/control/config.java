package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.util.MiniPID;

public class config {

    double defaultTurnP = 2, defaultTurnI = 0, defaultTurnD = 0;
    double defaultTurnMultiplier = 1;

    MiniPID defaultTurnPID = new MiniPID(defaultTurnP, defaultTurnI, defaultTurnD);

    double defaultXP = 1, defaultXI = 0, defaultXD = 0;

    MiniPID defaultXPID = new MiniPID(defaultXP, defaultXI, defaultXD);

    double defaultYP = 1, defaultYI = 0, defaultYD = 0;

    MiniPID defaultYPID = new MiniPID(defaultYP, defaultYI, defaultYD);
}
