package org.firstinspires.ftc.teamcode.subsystems;
import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Intake extends Subsystem {
    DcMotorEx intake;
    double ratio=1.0;
    public double getSetpoint() {
        return setpoint;
    }
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }
    double setpoint;
    @Override
    public void initialize(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setVelocity(0, AngleUnit.DEGREES);
        setpoint = 0;
    }

    @Override
    public void initPeriodic() {
    }

    @Override
    public void start() {}

    @Override
    public void runPeriodic() {
        intake.setVelocity( ratio * setpoint, AngleUnit.DEGREES);
    }

    @Override
    public void stop() {
        intake.setVelocity(0);
    }
}
