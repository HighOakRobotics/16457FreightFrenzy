package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainMecanum;
import org.firstinspires.ftc.teamcode.subsystems.positioning.Position;

import java.util.function.Supplier;

public class Mecanum extends Subsystem {
    DriveTrainMecanum mecanum;
    Supplier<Position> positionSupplier;

    public Mecanum() {
        // TODO deprecate after sensor system is in place
    }

    public Mecanum(Supplier<Position> positionSupplier) {
        this.positionSupplier = positionSupplier;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        priority = 20;
        mecanum = new DriveTrainMecanum(hardwareMap);
        //if (positionSupplier != null)
        //    mecanum.setLocalizer(new PositionLocalizer(positionSupplier));
        mecanum.setMotorPowers(0, 0, 0, 0);
        mecanum.update();
    }

    @Override
    public void initPeriodic() {
        mecanum.update();
    }

    @Override
    public void start() {
    }

    @Override
    public void runPeriodic() {
        mecanum.update();
    }

    @Override
    public void stop() {
        mecanum.setMotorPowers(0, 0, 0, 0);
        mecanum.update();
    }

    public DriveTrainMecanum mecanum() {
        return mecanum;
    }
}