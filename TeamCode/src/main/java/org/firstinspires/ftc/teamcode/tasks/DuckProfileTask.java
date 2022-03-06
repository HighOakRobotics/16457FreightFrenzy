package org.firstinspires.ftc.teamcode.tasks;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystems.Carousel;

public class DuckProfileTask extends Task {

    final double TRAVELLING_DISTANCE = 8;
    final double SLIDING_SETPOINT = 20;
    final double SLIDING_TIME = 0.5;

    final double VELOCITY_CONSTRAINT = 15 ;
    final double ACCELERATION_CONSTRAINT = 10;

    double direction;

    MotionProfile profile;
    Carousel carousel;

    State state;
    Clock clock;

    public DuckProfileTask(Carousel carousel, int direction) {
        this.profile = MotionProfileGenerator.generateMotionProfile(
                new MotionState(0,0),
                new MotionState(direction * TRAVELLING_DISTANCE, SLIDING_SETPOINT),
                (x) -> VELOCITY_CONSTRAINT,
                (x) -> ACCELERATION_CONSTRAINT
        );
        this.carousel = carousel;
        this.direction = direction;
        this.clock = new Clock();
    }

    @Override
    public void init() {
        state = State.TRAVELLING;
        carousel.setSetpoint(profile.start().getV());
        clock.startTiming();
        running = true;
    }

    @Override
    public void loop() {
        switch (state) {
            case TRAVELLING:
                carousel.setSetpoint(profile.get(clock.getSeconds()).getV());
                if (clock.getSeconds() > profile.duration()) {
                    state = State.SLIDING;
                    clock.startTiming();
                }
                break;
            case SLIDING:
                carousel.setSetpoint(direction * SLIDING_SETPOINT);
                if (clock.getSeconds() > SLIDING_TIME)
                    running = false;
                break;
        }
    }

    @Override
    public void stop(boolean interrupted) {
        carousel.setSetpoint(0);
    }

    public enum State {
        TRAVELLING, SLIDING
    }
}
