package org.firstinspires.ftc.teamcode.PedroPathing.pedro;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "TestRed2", group = "Autonomous")
public class OthersideRedautotest extends LinearOpMode {

    private Follower follower;
    private DcMotor shooter = null;
    private CRServo indexer;
    private DcMotor intake = null;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();


    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.4201, 132.2342, 270);
    private final Pose path1Start = poseFactory.of(59.4201, 132.2342, 270);
    private final Pose path1 = poseFactory.of(14.1255, 104.2693, 180);
    private final Pose path1Control1 = poseFactory.of(63.7055, 89.5701, 0);
    private final Pose path1Control2 = poseFactory.of(23.453, 112.5933, 0);


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
    public Command wait_time(int seconds){
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .setDone(() -> timer.seconds() >= seconds)
                .requiring(seconds);

    }
    public Command ShootBall(){
        return sequential(
                runIndexerForward(indexer),
                runIndexerBackward(indexer)
        );
    }
    public Command wait_until_tipped() {
        boolean[] isdown = {false};

        return Command.build()
                .setExecute(() -> {
                    aprilTagWebcam.update();
                    isdown[0] = aprilTagWebcam.is_red_scoring_down();
                })
                .setDone(() -> isdown[0]);
    }
    public Command autoRoutine() {
        return sequential(
                wait_until_tipped(),
                ShootBall(),
                ShootBall(),
                ShootBall(),
                follow(follower, path1())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        aprilTagWebcam.init(hardwareMap, telemetry);
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        indexer = hardwareMap.get(CRServo.class, "indexer");
        intake = hardwareMap.get(DcMotor.class, "intake_motor");

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        aprilTagWebcam.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            shooter.setPower(1);
            intake.setPower(0.7);

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


}

