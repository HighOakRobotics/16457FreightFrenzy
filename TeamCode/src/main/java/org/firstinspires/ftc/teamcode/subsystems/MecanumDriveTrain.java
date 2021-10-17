package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class MecanumDriveTrain extends Subsystem {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    @Override
    public void initialize(HardwareMap hardwareMap) {
        this.frontLeft=hardwareMap.get(DcMotorEx.class, "frontleft");
        this.frontRight=hardwareMap.get(DcMotorEx.class, "frontright");
        this.backLeft=hardwareMap.get(DcMotorEx.class, "backleft");
        this.backRight=hardwareMap.get(DcMotorEx.class, "backright");

    }
    public void drive(Gamepad gamepad){
        double drive = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;

        double frontLeftPower = Range.clip(drive + strafe + turn, -1.0,1.0) ;
        double frontRightPower = Range.clip(drive - strafe - turn, -1.0, 1.0);
        double backLeftPower = Range.clip(drive + strafe - turn, -1.0, 1.0);
        double backRightPower = Range.clip(drive - strafe + turn, -1.0, 1.0);

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
    @Override
    public void initPeriodic() {

    }

    @Override
    public void start() {

    }

    @Override
    public void runPeriodic() {

    }

    @Override
    public void stop() {

    }
}
