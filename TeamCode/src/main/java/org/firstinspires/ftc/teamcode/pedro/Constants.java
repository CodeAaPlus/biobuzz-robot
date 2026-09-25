package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
        c.xPodName.set("leftBack");
        c.yPodName.set("rightFront");
        c.imuName.set("imu");
        c.xPodOffset.set(-35.82227469603072);
        c.yPodOffset.set(-7.147893553501859);
        c.forwardTicksToInches.set(0.004064474295846891);
        c.strafeTicksToInches.set(0.010028923919450082);
        c.xPodDirection.set(Encoder.REVERSE);
        c.yPodDirection.set(Encoder.FORWARD);
        c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        )));
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.30417601556816787);
                Controller secondaryTranslationalForward = Controller.proportional(0.11238491001961144);
                Controller primaryTranslationalLateral = Controller.proportional(0.28395075812567894);
                Controller secondaryTranslationalLateral = Controller.proportional(0.10491221782344386);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.00946875468592251));
                c.brake.set(Controller.proportionalFeedforward(0.008048441483034133));

                c.headingFeedback.set(Controller.proportional(5.687653307185227));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0149716830323444, 0.001167985294702176));

                c.linearBrakeCoefficients.set(Matrix.diag(0.12580677137398785, 0.07195143904344103));
                c.quadraticBrakeCoefficients.set(Matrix.diag(5.144686885429859E-4, 4.398714019979766E-4));

                c.maxAchievableForwardVelocity.set(92.32500560355386);
                c.maxAchievableStrafeVelocity.set(140.4365085276347);
                c.naturalForwardDeceleration.set(28.961072151461202);
                c.naturalStrafeDeceleration.set(176.68857517003448);
            }
    );
}