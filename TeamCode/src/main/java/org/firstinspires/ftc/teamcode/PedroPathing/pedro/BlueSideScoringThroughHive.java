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

@Autonomous(name = "BlueSideScoringThroughHive", group = "Autonomous")
public class BlueSideScoringThroughHive extends LinearOpMode {
    private Follower follower;
    private DcMotorEx pollen_shooter = null;
    private DcMotor indexer = null;
    private DcMotor intake = null;
    private Servo left_servo = null;
    private Servo right_servo = null;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(79.4458, 129.9728, 90);
    private final Pose path1 = poseFactory.of(112.2511, 96.1958, 0);
    private final Pose path1Control1 = poseFactory.of(89.8015, 106.1955, 0);
    private final Pose point2 = poseFactory.of(126.2248, 95.8403, -1.4575);
    private final Pose point3 = poseFactory.of(118.5385, 96.1307, -2.164);
    private final Pose path3Start = poseFactory.of(118.5385, 96.1307, 0);
    private final Pose path3 = poseFactory.of(84.159, 18.5796, 270);
    private final Pose path3Control1 = poseFactory.of(106.8095, 12.7531, 0);
    private final Pose path4Start = poseFactory.of(84.159, 18.5796, 270);
    private final Pose path4 = poseFactory.of(130.1269, 22.6368, 270);





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
                .setStart(() -> pollen_shooter.setVelocity(1375))
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
        timer.reset();
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
                wait_time(3),
                follow(follower, path3()),
                moveflowerthingup(),
                follow(follower, path3_2()),
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
        return Paths.line(path1, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }

    public Path path3_2() {
        return Paths.curve(path3Start, path3Control1, path3).linear(path3Start, path3);
    }

    public Path path4() {
        return Paths.line(path4Start, path4).linear(path4Start, path4);
    }
}