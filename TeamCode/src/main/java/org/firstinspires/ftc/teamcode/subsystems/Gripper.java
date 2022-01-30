package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Gripper extends Subsystem {

    AngularServo servo;
    double target; // radians

    // all in radians
    double CLOSED_POSITION = 0;
    double INTAKE_POSITION = Math.PI / 6;
    double OPEN_POSITION = Math.PI / 3;

    public GripperState getState() {
        return state;
    }

    public void setState(GripperState state) {
        this.state = state;
    }

    GripperState state;

    @Override
    public void initialize(HardwareMap hardwareMap) {
        servo = new AngularServo(
                hardwareMap.get(Servo.class, "gripper"), 3 * Math.PI / 4);
        servo.setPosition(0);
        target = 0;
        state = GripperState.CLOSED;
    }

    @Override
    public void initPeriodic() { }

    @Override
    public void start() { }

    @Override
    public void runPeriodic() {
        switch (state) {
            case CLOSED:
                target = CLOSED_POSITION;
                break;
            case INTAKE:
                target = INTAKE_POSITION;
                break;
            case OPEN:
                target = OPEN_POSITION;
                break;
        }
        servo.setPosition(target);
    }

    @Override
    public void stop() {}

    public enum GripperState {
        CLOSED, INTAKE, OPEN
    }
}
