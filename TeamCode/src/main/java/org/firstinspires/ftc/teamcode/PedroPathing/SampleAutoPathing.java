package org.firstinspires.ftc.teamcode.PedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;


public class SampleAutoPathing extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //START POSITION_END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT\\

        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD
    }
    PathState pathstate;


    private final Pose startpose = new Pose(56, 8, Math.toRadians(90));
    private final Pose testpose = new Pose(56, 36, Math.toRadians(90));

    private PathChain driveStartShoot;

    public void buildPaths(){
        //put in coordinates for starting pose > ending pose
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startpose, testpose))
                .setLinearHeadingInterpolation(startpose.getHeading(), testpose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathstate) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartShoot, true);
                setPathState(PathState.SHOOT_PRELOAD); //reset the time & mmake new state
                break;
            case SHOOT_PRELOAD:
                //TODO add logic to shooter
                //check if follower dones it's path?
                if(!follower.isBusy()) {
                    telemetry.addLine("Path 1 Done");
                    //transition to next state
                }
                    break;
            default:
                telemetry.addLine("No State");
                break;
        }
    }
    public void setPathState(PathState newState){
        pathstate = newState;
        pathTimer.resetTimer();

    }
    @Override
    public void init() {
        pathstate = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        //TODO add in any other setups stuff

        buildPaths();
    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathstate);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathstate.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }
}
