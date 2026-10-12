package org.firstinspires.ftc.teamcode.PedroPathing.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left_drive");
        c.frontRightName.set("front_right_drive");
        c.backLeftName.set("back_left_drive");
        c.backRightName.set("back_right_drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(6.054607601616326);
        c.yPodOffset.set(-1.2618030337836799);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.19682474534825736);
                Controller secondaryTranslationalForward = Controller.proportional(0.07272148415212432);
                Controller primaryTranslationalLateral = Controller.proportional(0.2318672791241138);
                Controller secondaryTranslationalLateral = Controller.proportional(0.0856687640285557);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016575212656912346));
                c.brake.set(Controller.proportionalFeedforward(0.014088930758375494));

                c.headingFeedback.set(Controller.proportional(3.431440021355427));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.042914925575700626, 0.0031555163512336777));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07145532373517328, 0.0568077145179741));
                c.quadraticBrakeCoefficients.set(Matrix.diag(8.283345265735425E-4, 0.0010383894534746225));

                c.maxAchievableForwardVelocity.set(62.37272269650316);
                c.maxAchievableStrafeVelocity.set(52.18006742591429);
                c.naturalForwardDeceleration.set(32.50597685098606);
                c.naturalStrafeDeceleration.set(56.79320558135542);
            }
    );
}