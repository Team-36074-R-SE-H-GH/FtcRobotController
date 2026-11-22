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

@Autonomous(name = "RedSideHiveUpAroundHive", group = "Autonomous")
public class RedSideHiveUpAroundHive extends LinearOpMode {

    private Follower follower;
    private DcMotor shooter = null;
    private CRServo indexer;
    private DcMotor intake = null;
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
                ShootBall(),
                ShootBall(),
                ShootBall(),
                ShootBall(),
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                ShootBall(),
                ShootBall(),
                ShootBall(),
                ShootBall(),
                follow(follower, path4())
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