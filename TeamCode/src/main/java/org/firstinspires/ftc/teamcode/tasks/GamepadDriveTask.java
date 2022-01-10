package org.firstinspires.ftc.teamcode.tasks;

import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.StartEndTask;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainMecanum;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

public class GamepadDriveTask extends StartEndTask {


    public GamepadDriveTask(Gamepad gamepad, Mecanum drive) {
        super(() -> {
            drive.mecanum().setDriveDST(
                    () -> gamepad.right_bumper ? gamepad.left_stick_y : gamepad.left_stick_y * 0.5,
                    () -> gamepad.right_bumper ? gamepad.left_stick_x : gamepad.left_stick_x * 0.5,
                    () -> gamepad.right_bumper ? gamepad.right_stick_x : gamepad.right_stick_x * 0.5
            );
        }, () -> {
            drive.mecanum().idle();
        });
    }
}
