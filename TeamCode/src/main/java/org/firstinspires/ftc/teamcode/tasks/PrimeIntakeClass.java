package org.firstinspires.ftc.teamcode.tasks;

import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystems.legacy.LegacyArm;
import org.firstinspires.ftc.teamcode.subsystems.legacy.LegacyGripper;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class PrimeIntakeClass extends Task {

    LegacyArm arm;
    LegacyGripper legacyGripper;
    Intake intake;
    Clock clock;

    public PrimeIntakeClass(LegacyArm arm, LegacyGripper legacyGripper, Intake intake) {
        this.arm = arm;
        this.legacyGripper = legacyGripper;
        this.intake = intake;
        this.clock = new Clock();
    }

    @Override
    public void init() {
        running = true;
        arm.setMode(LegacyArm.ArmMode.INTAKE);
        legacyGripper.setState(LegacyGripper.GripperState.CLOSED);
        clock.startTiming();
    }

    @Override
    public void loop() {
        if (arm.getArmPosition() < 565 && arm.getWristPosition() < -400)
            running = false;
    }

    @Override
    public void stop(boolean interrupted) {
        intake.setSetpoint(-10);
        legacyGripper.setState(LegacyGripper.GripperState.OPEN);
    }
}
