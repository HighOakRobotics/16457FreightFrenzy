package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LegacyGripper extends Subsystem {
    Servo gripper;
    double kClose = 0.4;
    double kOpen = 0.65;

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
        //initialized to OPEN for teleop
        state = GripperState.CLOSED;
        gripper.setPosition(kClose);
    }


    @Override
    public void initPeriodic() {
        runPeriodic();
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
        gripper.setPosition(kClose);
    }

    public enum GripperState {
        OPEN, CLOSED;
    }
}
