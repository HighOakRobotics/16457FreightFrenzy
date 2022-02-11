package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@TeleOp
public class ArmEncoderTool extends SequoiaOpMode {

    Arm arm = new Arm();

    @Override
    public void initTriggers() {

    }

    @Override
    public void runTriggers() {
        arm.setArmState(Arm.ArmState.IDLE);
    }
}
