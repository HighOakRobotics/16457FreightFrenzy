package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.StartEndTask;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypointGraph;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.tasks.GamepadDriveTask;
import org.firstinspires.ftc.teamcode.tasks.GoToArmWaypointTask;

import java.util.concurrent.atomic.AtomicInteger;

@TeleOp(name = "TeleOp", group = "Working Title")
public class TeleOperated extends SequoiaOpMode {
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

        gamepad1H.upButton().onPress(new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.INTAKE_DOWN_READY));
        gamepad1H.rightButton().onPress(new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.RIGHT_TRACKING));
        gamepad1H.leftButton().onPress(new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.LEFT_TRACKING));
        gamepad1H.downButton().onPress(new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.BACK_TRACKING));

        gamepad1H.leftBumperToggleButton().risingWithCancel(new InstantTask(() -> {
            switch (arm.getGripperState()) {
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
        gamepad1H.rightBumperButton().onPress(new InstantTask(() -> arm.setGripperState(Arm.GripperState.OPEN)));

        //made intake basically a copy of rotator, hopefully that works
        gamepad1H.yToggleButton().risingWithCancel(new StartEndTask(() -> {
            intake.setSetpoint(-60);
        }, () -> {intake.setSetpoint(0);}));

        AtomicInteger rotationdir = new AtomicInteger(1);
        gamepad1H.aToggleButton().risingWithCancel(new StartEndTask(() -> carousel.setSetpoint(10 * rotationdir.get()), () -> carousel.setSetpoint(0)));
        gamepad1H.upButton().onPress(new InstantTask(() -> rotationdir.getAndUpdate(v -> 1)));
        gamepad1H.downButton().onPress(new InstantTask(() -> rotationdir.getAndUpdate(v -> -1)));
    }
}
