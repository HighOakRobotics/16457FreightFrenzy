package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.WaitTask;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@TeleOp
public class WristTestOpMode extends SequoiaOpMode {

    Arm arm = new Arm();

    @Override
    public void initTriggers() {

    }

    @Override
    public void runTriggers() {
        scheduler.schedule(new SequentialTaskBundle(
                new InstantTask(() -> {
                    arm.setArmState(Arm.ArmState.TARGET_POSITION);
                    arm.setWristState(Arm.WristState.TARGET);
                    arm.setWristTarget(0);
                }),
                new WaitTask(1),
                new InstantTask(() -> arm.setWristTarget(Math.PI/2)),
                new WaitTask(1),
                new InstantTask(() -> arm.setWristTarget(3 * Math.PI / 2 - Math.PI / 6))
        ));
    }
}
