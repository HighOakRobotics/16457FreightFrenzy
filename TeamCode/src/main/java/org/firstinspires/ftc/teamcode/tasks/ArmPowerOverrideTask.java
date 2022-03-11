package org.firstinspires.ftc.teamcode.tasks;

import com.ftc11392.sequoia.task.Task;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

public class ArmPowerOverrideTask extends Task {

    Arm arm;
    Gamepad gamepad;

    boolean didNotInitialize;

    public ArmPowerOverrideTask(Arm arm, Gamepad gamepad) {
        this.arm = arm;
        this.gamepad = gamepad;
    }

    @Override
    public void init() {
        didNotInitialize = false;
        if (arm.controlLocked()) {
            didNotInitialize = true;
            running = false;
            return;
        }
        arm.lockControl();
        arm.setArmState(Arm.ArmState.TARGET_POWER);
    }

    @Override
    public void loop() {
        if (didNotInitialize) return;
        arm.setArmTargetPower(gamepad.right_stick_y);
        arm.setRotatorTargetPower(gamepad.left_stick_x);
    }

    @Override
    public void stop(boolean interrupted) {
        if (didNotInitialize) return;
        arm.setArmTargetPower(0);
        arm.setRotatorTargetPower(0);
        arm.unlockControl();
    }
}
