package org.firstinspires.ftc.teamcode.opmodes.competition;


import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.WaitTask;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto Blue", group = "Working Title")
public class AutoBlue extends SequoiaOpMode {
    @Override
    public void initTriggers() {

    }

    @Override
    public void runTriggers() {
        scheduler.schedule(new SequentialTaskBundle(
                new WaitTask(365, TimeUnit.DAYS)
        ));
    }
}
