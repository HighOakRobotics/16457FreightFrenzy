package org.firstinspires.ftc.teamcode.opmodes.competition;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.StartEndTask;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Rotator;
import org.firstinspires.ftc.teamcode.tasks.GamepadDriveTask;
import org.firstinspires.ftc.teamcode.tasks.PrimeIntakeClass;

import java.util.concurrent.atomic.AtomicInteger;

@TeleOp(name = "TeleOp", group = "Working Title")
public class TeleOperated extends SequoiaOpMode {
    private final Mecanum drivetrain = new Mecanum();
    private final Arm arm = new Arm();
    private final Rotator rotator = new Rotator();
    private final Gripper gripper = new Gripper();
    private final Intake intake = new Intake();

    @Override
    public void initTriggers() {


    }

    @Override
    public void runTriggers() {
        gamepad1H.sticksButton(0.01).onPressWithCancel(new GamepadDriveTask(gamepad1, drivetrain));

        gamepad1H.leftButton().onPress(new InstantTask(() -> {
            arm.setMode(Arm.ArmMode.HORIZONTAL);
            intake.setSetpoint(0);
        }));
        gamepad1H.rightButton().onPress(new InstantTask(() -> {
            arm.setMode(Arm.ArmMode.VERTICAL);
            intake.setSetpoint(0);
        }));
        gamepad1H.leftBumperButton().onPress(new InstantTask(() -> {
            arm.setMode(Arm.ArmMode.HOME);
            gripper.setState(Gripper.GripperState.CLOSED);
            intake.setSetpoint(0);
        }));
        gamepad1H.rightBumperButton().onPress(new PrimeIntakeClass(arm, gripper, intake));

        gamepad1H.leftTriggerButton(0.01).whilePressed(new InstantTask(() -> arm.modifySetpoint(-0.75)));
        gamepad1H.rightTriggerButton(0.01).whilePressed(new InstantTask(() -> arm.modifySetpoint(0.75)));
/*
        gamepad1H.yToggleButton().risingWithCancel(new StartEndTask(() -> {
            intake.setVelocity(1800.0);
        }, () -> {
            intake.setVelocity(0);
        }));

*/
        //made intake basically a copy of rotator, hopefully that works
        gamepad1H.yButton().onPress(new InstantTask(() -> {
            intake.setSetpoint(10);
        }));

        AtomicInteger rotationdir = new AtomicInteger(1);
        gamepad1H.aToggleButton().risingWithCancel(new StartEndTask(() -> rotator.setSetpoint(10 * rotationdir.get()), () -> rotator.setSetpoint(0)));
        gamepad1H.upButton().onPress(new InstantTask(() -> rotationdir.getAndUpdate(v -> 1)));
        gamepad1H.downButton().onPress(new InstantTask(() -> rotationdir.getAndUpdate(v -> -1)));

        gamepad1H.bToggleButton().risingWithCancel(new StartEndTask(() -> gripper.setState(Gripper.GripperState.OPEN), () -> gripper.setState(Gripper.GripperState.CLOSED)));
    }
}
