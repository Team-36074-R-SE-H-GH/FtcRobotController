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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilTagWebcam;

@Autonomous(name = "BlueSideAudienceAroundHive", group = "Autonomous")
public class BlueSideAudienceAroundHive extends LinearOpMode {

    private Follower follower;
    private DcMotor shooter = null;
    private CRServo indexer;
    private DcMotor intake = null;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose start = poseFactory.of(82.8671, 14.866, 270);
    private final Pose path1 = poseFactory.of(93.9124, 14.8049, 270);
    private final Pose point2Start = poseFactory.of(93.9124, 14.8049, 0);
    private final Pose point2 = poseFactory.of(83.3875, 127.505, 92);
    private final Pose point2Control1 = poseFactory.of(139.5098, 83.885, 0);
    private final Pose point3 = poseFactory.of(126.4043, 44.5864, 105.2337);
    private final Pose point3Control1 = poseFactory.of(110.8516, 101.6969, 0);


    // Autonomous routine
    public Command runIndexerForward(CRServo indexer) {
        ElapsedTime timer = new ElapsedTime();

        return Command.build()
                .setStart(() -> timer.reset())
                .setExecute(() -> indexer.setPower(0.4))
                .setDone(() -> timer.seconds() >= 1)
                .setEnd(endCondition -> indexer.setPower(0))
                .requiring(indexer);
    }
    public Command runIndexerBackward(CRServo indexer) {
        ElapsedTime timer = new ElapsedTime();

        return Command.build()
                .setStart(() -> timer.reset())
                .setExecute(() -> indexer.setPower(-0.4))
                .setDone(() -> timer.seconds() >= 1)
                .setEnd(endCondition -> indexer.setPower(0))
                .requiring(indexer);
    }
    public Command ShootBall(){
        return sequential(
                runIndexerForward(indexer),
                runIndexerBackward(indexer)
        );
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


    public Command autoRoutine() {
        return sequential(
                wait_until_tippedba(),
                follow(follower, path1()),
                ShootBall(),
                follow(follower, path2()),
                ShootBall(),
                follow(follower, path3())


        );

    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        aprilTagWebcam.init(hardwareMap, telemetry);
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        indexer = hardwareMap.get(CRServo.class, "indexer");
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        shooter.setPower(0.76);
        intake.setPower(0.7);
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        aprilTagWebcam.update();

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

    //PATH FUNCTIONS GO HERE
    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3).reverseTangent();
    }
    }

