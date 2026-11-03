package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.ThreeWheelConfig;
import com.pedropathing.revhub.localizers.ThreeWheelLocalizer;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new ThreeWheelLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("bl");
        c.backRightName.set("br");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static ThreeWheelConfig localizerConfig = new ThreeWheelConfig(c -> {
        c.leftEncoderName.set("br");
        c.rightEncoderName.set("bl");
        c.strafeEncoderName.set("fl");
        c.leftPodY.set(4.3549959458227985);
        c.rightPodY.set(-4.190299294100751);
        c.strafePodX.set(-4.27097361714859);
        c.forwardTicksToInches.set(0.002973619489200135);
        c.strafeTicksToInches.set(0.002947316713741864);
        c.turnTicksToRadians.set(0.002909487021621254);
        c.leftEncoderDirection.set(Encoder.FORWARD);
        c.rightEncoderDirection.set(Encoder.REVERSE);
        c.strafeEncoderDirection.set(Encoder.REVERSE);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4864580451208012);
                Controller secondaryTranslationalForward = Controller.proportional(0.1797332492737755);
                Controller primaryTranslationalLateral = Controller.proportional(1.8007359962232408);
                Controller secondaryTranslationalLateral = Controller.proportional(0.6653238340524933);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.zero); //Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral)

                c.coast.set(Controller.proportionalFeedforward(0.014139985623614022));
                c.brake.set(Controller.proportionalFeedforward(0.012018987780071919));

                c.headingFeedback.set(Controller.proportional(0)); //0.25038185758823
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06700378422067145, 0.004844469373291129));

                c.linearBrakeCoefficients.set(
                        Matrix.diag(0.0, 0.0)
                );

                c.quadraticBrakeCoefficients.set(
                        Matrix.diag(0.0, 0.0)
                );

                c.maxAchievableForwardVelocity.set(73.18707579121386);
                c.maxAchievableStrafeVelocity.set(58.74527667268901);
                c.naturalForwardDeceleration.set(49.861008120128005);
                c.naturalStrafeDeceleration.set(107.51242649688359);
            }
    );

}