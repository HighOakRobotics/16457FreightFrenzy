package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AngularServo {
    Servo servo;
    double range; // In radians

    public AngularServo(Servo servo, double range) {
        this.servo = servo;
        this.range = range;
    }

    public void setPosition(double position) {
        double signal = Range.clip(position,0,range) / range;
        servo.setPosition(signal);
    }
}
