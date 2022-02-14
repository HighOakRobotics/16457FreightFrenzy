package org.firstinspires.ftc.teamcode.tasks;

import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.utils.Profile;
import org.firstinspires.ftc.teamcode.utils.ProfileGenerator;

public class DuckProfileTask extends Task {

    public final double SLIDING_TIME = 0;
    public final double SLIDING_SETPOINT = 0;

    Carousel carousel;
    Profile profile;
    DuckState duckState;
    Clock clock;
    int multiplier;

    public DuckProfileTask(Carousel carousel, int multiplier) {
        this.carousel = carousel;
        this.profile = new ProfileGenerator(10, 20)
                .generateProfile(multiplier * 7.5);
        this.clock = new Clock();
        this.multiplier = multiplier;
    }

    @Override
    public void init() {
        duckState = DuckState.TRAVELLING;
        carousel.setSetpoint(0);
        clock.startTiming();
        running = true;
    }

    @Override
    public void loop() {
        switch (duckState) {
            case TRAVELLING:
                carousel.setSetpoint(profile.getProfileVelocity(clock.getSeconds()));
                if (profile.isProfileComplete(clock.getSeconds())) {
                    duckState = DuckState.SLIDING;
                    clock.startTiming();
                }
                break;
            case SLIDING:
                carousel.setSetpoint(multiplier * SLIDING_SETPOINT);
                if (clock.getSeconds() > SLIDING_TIME) {
                    running = false;
                }
                break;
        }
    }

    @Override
    public void stop(boolean interrupted) {
        carousel.setSetpoint(0);
    }

    public enum DuckState {
        TRAVELLING, SLIDING
    }
}
