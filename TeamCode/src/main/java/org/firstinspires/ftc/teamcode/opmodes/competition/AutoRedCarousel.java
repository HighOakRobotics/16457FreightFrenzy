package org.firstinspires.ftc.teamcode.opmodes.competition;


import java.util.HashMap;
import java.util.concurrent.TimeUnit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.ParallelTaskBundle;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.SwitchTask;
import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.task.WaitTask;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypointGraph;
import org.firstinspires.ftc.teamcode.subsystems.DuckDetector;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.tasks.ArmTrackingTask;
import org.firstinspires.ftc.teamcode.tasks.AutoSlowDuckProfileTask;
import org.firstinspires.ftc.teamcode.tasks.LegacyDuckProfileTask;
import org.firstinspires.ftc.teamcode.tasks.FollowTrajectoryTask;
import org.firstinspires.ftc.teamcode.tasks.LegacyGoToArmWaypointTask;

@Autonomous(group = "CRI Red Auto")
public class AutoRedCarousel extends SequoiaOpMode {

    DuckDetector duckDetector = new DuckDetector(70, 160, 250);
    Mecanum mecanum = new Mecanum();
    Arm arm = new Arm();
    Carousel carousel = new Carousel();

    @Override
    public void initTriggers() {
        mecanum.mecanum().setPoseEstimate(new Pose2d(-60, -60, 0));
    }

    @Override
    public void runTriggers() {
        DuckDetector.DuckPipeline.DuckPosition position = duckDetector.getAnalysis();
        scheduler.schedule(new SequentialTaskBundle(
                new FollowTrajectoryTask(mecanum, new Pose2d(-60, -55, 0)),
                new FollowTrajectoryTask(mecanum, new Pose2d(-83, -65, 0)),
                new AutoSlowDuckProfileTask(carousel, 1),

                new FollowTrajectoryTask(mecanum, new Pose2d(-89, -36, Math.PI/2)),


                new InstantTask(this::requestOpModeStop)
        ));
    }
}
