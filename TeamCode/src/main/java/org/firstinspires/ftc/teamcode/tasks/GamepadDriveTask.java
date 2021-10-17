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
                    () -> gamepad.left_stick_y,
                    () -> gamepad.left_stick_x,
                    () -> gamepad.right_stick_x
            );
        }, () -> {
            drive.mecanum().idle();
        });
    }
}
