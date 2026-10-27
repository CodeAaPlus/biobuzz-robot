package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FixedMecanum extends Mecanum {

    public FixedMecanum(
            HardwareMap hardwareMap,
            MecanumConfig config) {
        super(hardwareMap, config);
    }

    @Override
    public double[] computeWheelPowersUnnormalized(
            DrivePowers powers) {

        double forward = powers.forward();
        double strafe = -powers.strafe();
        double turn = powers.turn();

        return new double[]{
            forward - strafe - turn,
            forward + strafe + turn,
            forward + strafe - turn,
            forward - strafe + turn
        };
    }
}
