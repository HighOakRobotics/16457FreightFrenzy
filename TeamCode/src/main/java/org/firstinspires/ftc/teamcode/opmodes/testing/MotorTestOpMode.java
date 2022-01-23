package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.WaitTask;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Mecanum;


@TeleOp(name = "Motor Cycle Test", group = "Working Title")
public class MotorTestOpMode extends SequoiaOpMode {

    DcMotorEx leftFront, leftBack, rightFront, rightBack;

    @Override
    public void initTriggers() {
        leftFront=hardwareMap.get(DcMotorEx.class, "frontleft");
        leftBack=hardwareMap.get(DcMotorEx.class, "backleft");
        rightFront=hardwareMap.get(DcMotorEx.class, "frontright");
        rightBack=hardwareMap.get(DcMotorEx.class, "backright");
    }

    @Override
    public void runTriggers() {
        scheduler.schedule(new SequentialTaskBundle(
                //left front
                new InstantTask(() -> {
                    leftFront.setPower(1);
                }),
                new WaitTask(2),
                //left back
                new InstantTask(() -> {
                    leftBack.setPower(1);
                }),
                new WaitTask(2),
                //right back
                new InstantTask(() -> {
                    rightBack.setPower(1);
                }),
                new WaitTask(2),
                //right front
                new InstantTask(() -> {
                    rightFront.setPower(1);
                }),
                new WaitTask(2),
                new InstantTask(() -> {
                    leftBack.setPower(0);
                    leftFront.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                }),
                new WaitTask(2)

        ));
    }
}
