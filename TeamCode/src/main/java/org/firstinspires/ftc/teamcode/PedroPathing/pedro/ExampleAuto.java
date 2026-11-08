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

@Autonomous(name = "ExampleAuto", group = "Autonomous")
public class ExampleAuto extends LinearOpMode {

    private Follower follower;
    private DcMotor shooter = null;
    private CRServo indexer;
    private DcMotor intake = null;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private final PoseFactory poseFactory = PoseFactory.degrees();



    private final Pose start = poseFactory.of(56.1753, 7.737, 90);
    private final Pose path1 = poseFactory.of(56.1753, 21.1837, 90);
    private final Pose point2 = poseFactory.of(7.6231, 8.3133, 180);
    private final Pose point2Segment1Start = poseFactory.of(7.6231, 8.3133, 90);
    private final Pose point2Segment1End = poseFactory.of(7.6231, 8.3133, 180);
    private final Pose point2Segment2Start = poseFactory.of(7.6231, 8.3133, 180);
    private final Pose point2Segment2End = poseFactory.of(7.6231, 8.3133, 180);
    private final Pose point3 = poseFactory.of(63.7604, 134.0193, 270);
    private final Pose point3Control1 = poseFactory.of(19.7123, 58.1583, 0);
    private final Pose point3Control2 = poseFactory.of(15.2728, 106.1075, 0);
    private final Pose point3Control3 = poseFactory.of(24.1004, 133.6244, 0);
    private final Pose point3Control4 = poseFactory.of(62.4506, 114.8389, 0);
    private final Pose point3Segment1Heading = poseFactory.of(63.7604, 134.0193, 180);
    private final Pose point3Segment2Start = poseFactory.of(63.7604, 134.0193, 180);
    private final Pose point3Segment2End = poseFactory.of(63.7604, 134.0193, 270);
    private final Pose point4 = poseFactory.of(14.7784, 118.7473, 270);
    private final Pose point4Control1 = poseFactory.of(58.5509, 102.1055, 0);
    private final Pose point4Control2 = poseFactory.of(52.011, 101.0763, 0);


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
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
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
        return Paths.line(start, path1).tangent();
    }

    public Path path2() {
        return Paths.line(path1, point2).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(point2Segment1Start, point2Segment1End)).until(1, Interpolator.linear(point2Segment2Start, point2Segment2End)));
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3Control2, point3Control3, point3Control4, point3).heading(Interpolator.piecewise().until(0.5, Interpolator.constant(point3Segment1Heading)).until(1, Interpolator.linear(point3Segment2Start, point3Segment2End)));
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4Control2, point4).constant(point4);
    }
}
