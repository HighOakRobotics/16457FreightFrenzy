package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpBlue")
public class TeleOperatedBlue extends TeleOperatedCommon{

    @Override
    protected int getDuckMultiplier() {
        return 1;
    }
}
