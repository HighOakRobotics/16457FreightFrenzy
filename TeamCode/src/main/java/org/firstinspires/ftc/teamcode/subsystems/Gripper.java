package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper extends Subsystem {
    Servo gripper;
    double kClose = 0;
    double kOpen = 1;

    public GripperState getState() {
        return state;
    }

    public void setState(GripperState state) {
        this.state = state;
    }

    GripperState state;

    @Override
    public void initialize(HardwareMap hardwareMap) {
        gripper = hardwareMap.get(Servo.class, "gripper");
        state = GripperState.CLOSED;
        gripper.setPosition(0);
    }

    @Override
    public void initPeriodic() {
    }

    @Override
    public void start() {
    }

    @Override
    public void runPeriodic() {
        switch (state) {
            case OPEN:
                gripper.setPosition(kOpen);
                break;
            case CLOSED:
                gripper.setPosition(kClose);
                break;
        }
    }

    @Override
    public void stop() {

    }

    public enum GripperState {
        OPEN, CLOSED;
    }
}
