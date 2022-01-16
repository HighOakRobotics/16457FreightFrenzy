package org.firstinspires.ftc.teamcode.opmodes.testing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DriveTrainTest", group="Working Title")
public class TallDriveTrainTest extends LinearOpMode {
    DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("frontleft");
        rightFront = hardwareMap.dcMotor.get("frontright");
        leftBack = hardwareMap.dcMotor.get("backleft");
        rightBack = hardwareMap.dcMotor.get("backright");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            double d=gamepad1.left_stick_y;
            double s=gamepad1.left_stick_x;
            double t=gamepad1.right_stick_x;
            t=t*0.5;
            double leftFrontpower=d + s - t;
            double leftBackpower=d - s - t;
            double rightBackpower=d + s + t;
            double rightFrontpower=d - s + t;



            leftFront.setPower(Range.clip(leftFrontpower, -1.0, 1.0));
            leftBack.setPower(Range.clip(leftBackpower, -1.0, 1.0));
            rightFront.setPower(Range.clip(rightFrontpower, -1.0, 1.0));
            rightBack.setPower(Range.clip(rightBackpower, -1.0, 1.0));
            idle();
        }

    }
}

/*
@Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }



     double v = d - s - t;
        double v1 = d + s - t;
        double v2 = d - s + t;
        double v3 = d + s + t;
 */


