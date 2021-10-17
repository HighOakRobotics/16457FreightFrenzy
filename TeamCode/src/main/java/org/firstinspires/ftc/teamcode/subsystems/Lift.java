package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.PIDFSubsystem;
import com.ftc11392.sequoia.subsystem.Subsystem;
import com.ftc11392.sequoia.util.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

public class Lift extends PIDFSubsystem {
    private DcMotorEx lift;

    public Lift() {
        super(new PIDFController(1, 1, 1));
    }

    @Override
    protected void useOutput(double output) {
        lift.setPower(Range.clip(output, -1, 1));

    }

    @Override
    protected double getFeedback() {
        return lift.getCurrentPosition();
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        this.lift=hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(0);

    }
}
