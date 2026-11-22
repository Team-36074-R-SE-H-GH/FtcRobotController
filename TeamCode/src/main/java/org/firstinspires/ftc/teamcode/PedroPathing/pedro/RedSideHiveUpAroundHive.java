package org.firstinspires.ftc.teamcode.PedroPathing.pedro;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilTagWebcam;

@Autonomous(name = "RedSideHiveUpAroundHive", group = "Autonomous")
public class RedSideHiveUpAroundHive extends LinearOpMode {

    private Follower follower;
    private DcMotorEx pollen_shooter = null;
    private DcMotor indexer = null;
    private DcMotor intake = null;
    private Servo left_servo = null;
    private Servo right_servo = null;

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.4337, 9.4337, 90);
    private final Pose path1 = poseFactory.of(15.2756, 47.4693, 180);
    private final Pose path1Control1 = poseFactory.of(47.074, 45.7891, 0);
    private final Pose path1Segment1Start = poseFactory.of(15.2756, 47.4693, 90);
    private final Pose path1Segment1End = poseFactory.of(15.2756, 47.4693, 180);
    private final Pose path1Segment2Start = poseFactory.of(15.2756, 47.4693, 180);
    private final Pose path1Segment2End = poseFactory.of(15.2756, 47.4693, 180);
    private final Pose point2 = poseFactory.of(24.4013, 47.2257, 178.4704);
    private final Pose point3 = poseFactory.of(57.6779, 126.5631, -270);
    private final Pose point3Control1 = poseFactory.of(32.8113, 56.718, 0);
    private final Pose point3Control2 = poseFactory.of(20.8304, 119.7622, 0);
    private final Pose point3Control3 = poseFactory.of(34.9404, 105.4979, 0);
    private final Pose point3Segment2Start = poseFactory.of(57.6779, 126.5631, -91.6624);
    private final Pose point3Segment2End = poseFactory.of(57.6779, 126.5631, 90);
    private final Pose point4 = poseFactory.of(11.2486, 116.9195, 11.7338);


    // Autonomous routine
    public Command runIndexerForward() {

        return Command.build()
                .setStart(() -> indexer.setPower(0.8))
                .setDone(() -> true);

    }
    public Command runIndexerBackward() {

        return Command.build()
                .setStart(() -> indexer.setPower(-0.8))
                .setDone(() -> true);
    }
    public Command startPollenShooter(){
        return Command.build()
                .setStart(() -> pollen_shooter.setVelocity(1390))
                .setDone(() -> true);
    }
    public Command stopPollenShooter(){
        return Command.build()
                .setStart(() -> pollen_shooter.setVelocity(0))
                .setDone(() -> true);
    }
    public Command moveflowerthingdown() {
        return Command.build()
                .setStart(() -> {
                    left_servo.setPosition(0.2);
                    right_servo.setPosition(0.8);
                })
                .setDone(() -> true);
    }
    public Command moveflowerthingup() {
        return Command.build()
                .setStart(() -> {
                    left_servo.setPosition(1);
                    right_servo.setPosition(0);
                })
                .setDone(() -> true);
    }

    public Command wait_until_tippedrs() {
        boolean[] is_down = {false};

        return Command.build()
                .setExecute(() -> {
                    aprilTagWebcam.update();
                    is_down[0] = aprilTagWebcam.is_red_scoring_down();
                })
                .setDone(() -> is_down[0]);
    }
    public Command wait_until_tippedra() {
        boolean[] is_down = {false};

        return Command.build()
                .setExecute(() -> {
                    aprilTagWebcam.update();
                    is_down[0] = aprilTagWebcam.is_red_audience_down();
                })
                .setDone(() -> is_down[0]);
    }

    public Command wait_until_tippedbs() {
        boolean[] is_down = {false};

        return Command.build()
                .setExecute(() -> {
                    aprilTagWebcam.update();
                    is_down[0] = aprilTagWebcam.is_blue_scoring_down();
                })
                .setDone(() -> is_down[0]);
    }

    public Command wait_until_tippedba() {
        boolean[] is_down = {false};

        return Command.build()
                .setExecute(() -> {
                    aprilTagWebcam.update();
                    is_down[0] = aprilTagWebcam.is_blue_audience_down();
                })
                .setDone(() -> is_down[0]);
    }
    public Command wait_time(double seconds){
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .setDone(() -> timer.seconds() >= seconds)
                .requiring(seconds);

    }


    public Command autoRoutine() {
        return sequential(
                startPollenShooter(),
                wait_time(1.75),
                runIndexerForward(),
                wait_time(6),
                stopPollenShooter(),
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                startPollenShooter(),
                wait_time(1.75),
                runIndexerForward(),
                wait_time(6),
                stopPollenShooter(),
                follow(follower, path4())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        aprilTagWebcam.init(hardwareMap, telemetry);
        pollen_shooter = hardwareMap.get(DcMotorEx.class, "pollen_shooter");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        left_servo = hardwareMap.get(Servo.class, "left_servo");
        right_servo = hardwareMap.get(Servo.class, "right_servo");
        intake.setPower(0.7);
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        aprilTagWebcam.update();
        pollen_shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollen_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_servo.setPosition(1);
        right_servo.setPosition(0);
        waitForStart();
        schedule(autoRoutine());

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).heading(Interpolator.piecewise().until(0.7, Interpolator.linear(path1Segment1Start, path1Segment1End)).until(1, Interpolator.linear(path1Segment2Start, path1Segment2End)));
    }

    public Path path2() {
        return Paths.line(path1, point2).reverseTangent();
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3Control2, point3Control3, point3).heading(Interpolator.piecewise().until(0.3, Interpolator.tangent.reverse()).until(1, Interpolator.linear(point3Segment2Start, point3Segment2End)));
    }

    public Path path4() {
        return Paths.line(point3, point4).reverseTangent();
    }
}