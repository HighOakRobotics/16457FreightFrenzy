package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.StartEndTask;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypointGraph;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.tasks.ArmTrackingTask;
import org.firstinspires.ftc.teamcode.tasks.DuckProfileTask;
import org.firstinspires.ftc.teamcode.tasks.GamepadDriveTask;
import org.firstinspires.ftc.teamcode.tasks.GoToArmWaypointTask;

public abstract class TeleOperatedCommon extends SequoiaOpMode {
    private final Mecanum drivetrain = new Mecanum();
    private final Carousel carousel = new Carousel();
    private final Intake intake = new Intake();
    private final Arm arm = new Arm();

    @Override
    public void initTriggers() {
    }

    @Override
    public void runTriggers() {
        gamepad1H.sticksButton(0.01).onPressWithCancel(new GamepadDriveTask(gamepad1, drivetrain));

        gamepad1H.leftTriggerButton(0.01).onPress(new ArmTrackingTask(arm, () -> gamepad1.right_trigger, () -> gamepad1.left_trigger));
        gamepad1H.rightTriggerButton(0.01).onPress(new ArmTrackingTask(arm, () -> gamepad1.right_trigger, () -> gamepad1.left_trigger));

        gamepad1H.upButton().onPress(new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.INTAKE_DOWN_READY));
        gamepad1H.rightButton().onPress(new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.RIGHT_TRACKING));
        gamepad1H.leftButton().onPress(new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.LEFT_TRACKING));
        gamepad1H.downButton().onPress(new InstantTask(() -> {
            if (arm.getLastWaypoint() == ArmWaypointGraph.ArmWaypointName.BACK_DOWN_READY)
                scheduler.schedule(new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.BACK_TRACKING));
            else
                scheduler.schedule(new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.BACK_DOWN_READY));
        }));

        // PANIC! control of the arm is blocked because of a Task's failure to unlock control.
        gamepad2H.bButton().onPress(new InstantTask(arm::unlockControl));

        gamepad1H.leftBumperButton().onPress(new InstantTask(() -> {
            if (arm.getLastWaypoint() == ArmWaypointGraph.ArmWaypointName.INTAKE_DOWN_READY)
                scheduler.schedule(new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.INTAKE_DOWN_UPRIGHT));
            else if (arm.getLastWaypoint() == ArmWaypointGraph.ArmWaypointName.INTAKE_DOWN_UPRIGHT)
                scheduler.schedule(new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.INTAKE_DOWN_READY));
            else switch (arm.getGripperState()) {
                    case INTAKE:
                    case ELEMENT:
                    case TARGET:
                    case OPEN:
                        arm.setGripperState(Arm.GripperState.CLOSE);
                        break;
                    case CLOSE:
                        arm.setGripperState(Arm.GripperState.OPEN);
                        break;
                }
        }));

        //made intake basically a copy of rotator, hopefully that works
        gamepad1H.aToggleButton().risingWithCancel(new StartEndTask(() -> {
            intake.setSetpoint(-60);
        }, () -> {
            intake.setSetpoint(0);
        }));

        gamepad1H.yButton().onPress(new DuckProfileTask(carousel, getDuckMultiplier()));
    }

    protected abstract int getDuckMultiplier();
}