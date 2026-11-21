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
        new Foresight(foresightConfig));
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left_drive");
        c.frontRightName.set("front_right_drive");
        c.backLeftName.set("back_left_drive");
        c.backRightName.set("back_right_drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(4.622773898868111);
        c.yPodOffset.set(-4.164670208307702);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3031330634637512);
                Controller secondaryTranslationalForward = Controller.proportional(0.11199956708522296);
                Controller primaryTranslationalLateral = Controller.proportional(0.45561836154044755);
                Controller secondaryTranslationalLateral = Controller.proportional(0.16833881024235683);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01768519454971639));
                c.brake.set(Controller.proportionalFeedforward(0.01503241536725893));

                c.headingFeedback.set(Controller.proportional(4.874444405962536));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04356932184672816, 0.005067963178292262));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09274287481863282, 0.04609511038027414));
                c.quadraticBrakeCoefficients.set(Matrix.diag(7.283660629361645E-4, 0.0017695403984286202));

                c.maxAchievableForwardVelocity.set(59.406130055154826);
                c.maxAchievableStrafeVelocity.set(47.59386626741501);
                c.naturalForwardDeceleration.set(32.296905148337395);
                c.naturalStrafeDeceleration.set(65.66317028870488);
            }
    );
}