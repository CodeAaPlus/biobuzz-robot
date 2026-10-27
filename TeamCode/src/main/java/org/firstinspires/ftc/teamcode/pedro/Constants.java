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
        c.frontRightName.set("br"); // Remember to unswap
        c.backLeftName.set("bl");
        c.backRightName.set("fr"); // Remember to unswap
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static ThreeWheelConfig localizerConfig = new ThreeWheelConfig(c -> {
        c.leftEncoderName.set("br");
        c.rightEncoderName.set("bl");
        c.strafeEncoderName.set("fl");
        c.leftPodY.set(4.603933988368056);
        c.rightPodY.set(-3.147220499860984);
        c.strafePodX.set(-5.088580098718056);
        c.forwardTicksToInches.set(0.002973619489200135);
        c.strafeTicksToInches.set(0.002947316713741864);
        c.turnTicksToRadians.set(0.0026691844784676013);
        c.leftEncoderDirection.set(Encoder.FORWARD);
        c.rightEncoderDirection.set(Encoder.REVERSE);
        c.strafeEncoderDirection.set(Encoder.REVERSE);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(1.3154302879171047);
                Controller secondaryTranslationalForward = Controller.proportional(0.4860163424407298);
                Controller primaryTranslationalLateral = Controller.proportional(2.6218461576982968);
                Controller secondaryTranslationalLateral = Controller.proportional(0.9687020982499283);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01497431130597164));
                c.brake.set(Controller.proportionalFeedforward(0.012728164610075893));

                c.headingFeedback.set(Controller.proportional(14.201941246246868));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.10506771792529994, 0.0168865952721657));

                c.linearBrakeCoefficients.set(Matrix.diag(3.1326377851097966, -0.008117859999727728));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-0.05218252303601968, 0.002387593044910967));

                c.maxAchievableForwardVelocity.set(75.03878938105062);
                c.maxAchievableStrafeVelocity.set(66.96793454936157);
                c.naturalForwardDeceleration.set(89.35324062044607);
                c.naturalStrafeDeceleration.set(109.60567884151567);
            }
    );

}
