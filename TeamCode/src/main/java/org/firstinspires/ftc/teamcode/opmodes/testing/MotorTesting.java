package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="Motor Testing", group="Working Title")
public class MotorTesting extends LinearOpMode{
    DcMotorEx carousel;
    @Override
    public void runOpMode() throws InterruptedException {
        carousel=hardwareMap.get(DcMotorEx.class, "rotator");
        waitForStart();
        while (opModeIsActive()) {
            double power = gamepad1.right_stick_x;
            carousel.setPower(0.5 * power);
            idle();
        }
    }
}
