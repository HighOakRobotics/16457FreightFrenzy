package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.WaitTask;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

@TeleOp(name = "test", group = "Quackology")
public class MotorTestOpMode extends SequoiaOpMode {

    Mecanum drivetrain = new Mecanum();

    @Override
    public void initTriggers() {

    }

    @Override
    public void runTriggers() {
        scheduler.schedule(new SequentialTaskBundle(
                new InstantTask(() -> {
                    drivetrain.mecanum().setMotorPowers(1,0,0,0);
                }),
                new WaitTask(2),
                new InstantTask(() -> {
                    drivetrain.mecanum().setMotorPowers(0,1,0,0);
                }),
                new WaitTask(2),
                new InstantTask(() -> {
                    drivetrain.mecanum().setMotorPowers(0,0,1,0);
                }),
                new WaitTask(2),
                new InstantTask(() -> {
                    drivetrain.mecanum().setMotorPowers(0,0,0,1);
                }),
                new WaitTask(2),
                new InstantTask(() -> {
                    drivetrain.mecanum().setMotorPowers(0,0,0,0);
                }),
                new WaitTask(2)

        ));
    }
}
