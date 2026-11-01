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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilTagWebcam;

@Autonomous(name = "detectfliptest", group = "Autonomous")
public class detectfliptest extends LinearOpMode {

    private Follower follower;
    private DcMotor shooter = null;
    private CRServo indexer;
    private DcMotor intake = null;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(55.737, 115.254, 270);
    private final Pose path1Control1 = poseFactory.of(10.0821, 22.6628, 0);
    private final Pose path1Control2 = poseFactory.of(9.6638, 95.5854, 0);
    private final Pose point2Start = poseFactory.of(55.737, 115.254, 270);
    private final Pose point2 = poseFactory.of(87.7099, 115.3343, 270);
    private final Pose point3Start = poseFactory.of(87.7099, 115.3343, 270);
    private final Pose point3 = poseFactory.of(85.0809, 20.1225, 90);
    private final Pose point3Control1 = poseFactory.of(127.014, 95.6725, 0);
    private final Pose point3Control2 = poseFactory.of(129.6088, 33.6817, 0);
    private final Pose point4 = poseFactory.of(14.5591, 105.9585, 90);
    private final Pose point4Control1 = poseFactory.of(41.3622, 8.0329, 0);
    private final Pose point4Control2 = poseFactory.of(7.0915, 67.0541, 0);


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
        boolean[] isdown = {false};

        return Command.build()
                .setExecute(() -> {
                    aprilTagWebcam.update();
                    isdown[0] = aprilTagWebcam.is_red_scoring_down();
                })
                .setDone(() -> isdown[0]);
    }
    public Command wait_until_tippedra() {
        boolean[] isdown = {false};

        return Command.build()
                .setExecute(() -> {
                    aprilTagWebcam.update();
                    isdown[0] = aprilTagWebcam.is_red_audience_down();
                })
                .setDone(() -> isdown[0]);
    }

    public Command wait_until_tippedbs() {
        boolean[] isdown = {false};

        return Command.build()
                .setExecute(() -> {
                    aprilTagWebcam.update();
                    isdown[0] = aprilTagWebcam.is_blue_scoring_down();
                })
                .setDone(() -> isdown[0]);
    }

    public Command wait_until_tippedba() {
        boolean[] isdown = {false};

        return Command.build()
                .setExecute(() -> {
                    aprilTagWebcam.update();
                    isdown[0] = aprilTagWebcam.is_blue_audience_down();
                })
                .setDone(() -> isdown[0]);
    }


    public Command autoRoutine() {
        return sequential(
                wait_until_tippedra(),
                follow(follower, path1()),
                wait_until_tippedrs(),
                follow(follower, path2()),
                wait_until_tippedbs(),
                follow(follower, path3()),
                wait_until_tippedba(),
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
        return Paths.curve(start, path1Control1, path1Control2, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3Control2, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4Control2, point4).constant(point4);
    }
}
