package org.firstinspires.ftc.teamcode.opmodes.competition;


import java.util.HashMap;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.SwitchTask;
import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.task.WaitTask;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.DuckDetector;

@Autonomous(name = "Auto Red", group = "Working Title")
public class AutoRed extends SequoiaOpMode {

    DuckDetector duckDetector = new DuckDetector();

    @Override
    public void initTriggers() {

    }

    @Override
    public void runTriggers() {
        AtomicReference<DuckDetector.DuckPipeline.DuckPosition> position = new AtomicReference<>(DuckDetector.DuckPipeline.DuckPosition.LEFT);
        scheduler.schedule(new SequentialTaskBundle(
                new InstantTask(() -> {
                    position.set(duckDetector.getAnalysis());
                    duckDetector.stop();
                }),
                new SwitchTask(new HashMap<Object, Task>(){{
                    put(DuckDetector.DuckPipeline.DuckPosition.LEFT, new WaitTask(1));
                    put(DuckDetector.DuckPipeline.DuckPosition.CENTER, new WaitTask(1));
                    put(DuckDetector.DuckPipeline.DuckPosition.RIGHT, new WaitTask(1));
                }}, position::get),
                new InstantTask(this::requestOpModeStop)
        ));
    }
}
