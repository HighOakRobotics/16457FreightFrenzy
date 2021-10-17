package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Raymond", group = "Quackology")
public class Mech extends OpMode {
    private DcMotorEx FrontLeft, FrontRight, BackLeft, BackRight;
    @Override
    public void init() {
        FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        //drive, turn
        double drive = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;

        double frontLeftPower = Range.clip(drive+turn, -1.0, 1.0);
        double frontRightPower = Range.clip(drive-turn, -1.0, 1.0);
        double backLeftPower = Range.clip( drive+turn, -1.0, 1.0);
        double backRightPower = Range.clip(drive-turn, -1.0, 1.0);

        FrontLeft.setPower(frontLeftPower);
        FrontRight.setPower(frontRightPower);
        BackLeft.setPower(backLeftPower);
        BackRight.setPower(backRightPower);

    }
}
