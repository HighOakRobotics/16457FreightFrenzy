package org.firstinspires.ftc.teamcode.tasks;

import com.ftc11392.sequoia.task.Scheduler;
import com.ftc11392.sequoia.task.Task;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

import java.lang.reflect.Field;
import java.util.List;

public class ArmWatchdogTask extends Task {

    Arm arm;
    Field scheduledTasksField;

    public ArmWatchdogTask(Arm arm) {
        this.arm = arm;
        try {
            this.scheduledTasksField = Scheduler.class.getDeclaredField("scheduledTasks");
        } catch (NoSuchFieldException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void init() {
        scheduledTasksField.setAccessible(true);
        running = true;
    }

    @Override
    public void loop() {
        boolean noSchedule = false;
        try {
            if (((List) scheduledTasksField.get(Scheduler.getInstance())).size() == 1 && arm.controlLocked()) {
                arm.unlockControl();
                telemetry.log().add("arm watchdog activated! resolved locks...");
            }

        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void stop(boolean interrupted) {

    }
}
