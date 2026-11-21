package org.firstinspires.ftc.teamcode.PedroPathing.pedro;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
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

@Autonomous(name = "RedSideHiveUpThroughHive", group = "Autonomous")
public class RedSideHiveUpThroughHive extends LinearOpMode {

    private Follower follower;
    private DcMotorEx pollen_shooter = null;
    private DcMotor indexer = null;
    private DcMotor intake = null;
    private Servo left_servo = null;
    private Servo right_servo = null;

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose path1 = poseFactory.of(11.7471, 47.1977, 180);
    private final Pose path1Control1 = poseFactory.of(49.5797, 42.1672, 0);
    private final Pose path2Start = poseFactory.of(11.7471, 47.1977, 180);
    private final Pose path2 = poseFactory.of(21.6359, 46.9656, 180);
    private final Pose path3 = poseFactory.of(60.0917, 132.4259, -90);
    private final Pose path3Control1 = poseFactory.of(77.4963, 0, 0);
    private final Pose path3Segment1Heading = poseFactory.of(60.0917, 132.4259, 180);
    private final Pose path3Segment3Start = poseFactory.of(60.0917, 132.4259, -94.7554);
    private final Pose path3Segment3End = poseFactory.of(60.0917, 132.4259, 270);
    private final Pose path4 = poseFactory.of(12.4504, 116.6555, -270);
    private final Pose path4Control1 = poseFactory.of(45.7059, 93.2345, 0);
    private final Pose path4Segment1Start = poseFactory.of(12.4504, 116.6555, 270);
    private final Pose path4Segment1End = poseFactory.of(12.4504, 116.6555, 270);
    private final Pose path4Segment2Start = poseFactory.of(12.4504, 116.6555, -90);
    private final Pose path4Segment2End = poseFactory.of(12.4504, 116.6555, 90);

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
                moveflowerthingdown(),
                follow(follower, path1()),
                follow(follower, path2()),
                moveflowerthingup(),
                follow(follower, path3()),
                startPollenShooter(),
                wait_time(1),
                runIndexerForward(),
                wait_time(6),
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
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path2Start, path2).linear(path2Start, path2);
    }

    public Path path3() {
        return Paths.curve(path2, path3Control1, path3).heading(Interpolator.piecewise().until(0.1803, Interpolator.constant(path3Segment1Heading)).until(0.5, Interpolator.tangent.reverse()).until(1, Interpolator.linear(path3Segment3Start, path3Segment3End)));
    }

    public Path path4() {
        return Paths.curve(path3, path4Control1, path4).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(path4Segment1Start, path4Segment1End)).until(1, Interpolator.linear(path4Segment2Start, path4Segment2End)));
    }
}