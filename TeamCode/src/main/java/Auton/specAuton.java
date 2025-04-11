package Auton;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Autonomous(name = "Specimen", group = "Auto")
public class specAuton extends OpMode{
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int scoreCount = 0;
    private int pathState;

    private final Pose startPose = new Pose(10.2,63.2,Math.toRadians(0));

    private final Pose scorePose = new Pose(37.9,69+scoreCount,Math.toRadians(0));

    private final Pose spike1Pose = new Pose(13.3,26, Math.toRadians(180));
    private final Pose spike1control1 = new Pose(9.6,16.1);
    private final Pose spike1control2 = new Pose(63.9,52.2);
    private final Pose spike1control3 = new Pose(126.5,20.6);

    private final Pose spike2Pose = new Pose(13.5,12.9,Math.toRadians(180));
    private final Pose spike2control1 = new Pose(115,18.7);

    private final Pose spike3Pose = new Pose(10.1,7.9,Math.toRadians(180));
    private final Pose spike3control1 = new Pose(114,13.3);
    private final Pose spike3control2 = new Pose(56,8.2);

    private final Pose grabPose = new Pose(10.1,32,Math.toRadians(180));

    private final Pose parkPose = new Pose(22.9,46,Math.toRadians(225));

    private PathChain scorePreload, pushSpike1, pushSpike2,pushSpike3,scoreFirst, scorePickup,scoreDropoff,park;

    public void buildPaths(){
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose),new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading())
                .build();

        pushSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose),new Point(spike1control1),new Point(spike1control2),new Point(spike1control3),new Point(spike1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),spike1Pose.getHeading())
                .build();

        pushSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(spike1Pose),new Point(spike2control1),new Point(spike2Pose)))
                .setLinearHeadingInterpolation(spike1Pose.getHeading(),spike2Pose.getHeading())
                .build();

        pushSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(spike2Pose),new Point(spike3control1),new Point(spike3control2), new Point(spike3Pose)))
                .setLinearHeadingInterpolation(spike2Pose.getHeading(),spike3Pose.getHeading())
                .build();

        scoreFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spike3Pose),new Point(scorePose)))
                .setLinearHeadingInterpolation(spike3Pose.getHeading(),scorePose.getHeading())
                .build();

        scorePickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose),new Point(grabPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),grabPose.getHeading())
                .build();

        scoreDropoff = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPose),new Point(scorePose)))
                .setLinearHeadingInterpolation(grabPose.getHeading(),scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose),new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),parkPose.getHeading())
                .build();


    }
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()){
                    /*score preload*/
                    follower.followPath(pushSpike1);
                    setPathState(2);

                }
                break;
            case 2:
                if (!follower.isBusy()){

                    follower.followPath(pushSpike2);
                    setPathState(3);

                }
                break;
            case 3:
                if (!follower.isBusy()){

                    follower.followPath(pushSpike3);
                    setPathState(4);

                }
                break;
            case 4:
                if (!follower.isBusy()){

                    follower.followPath(scoreFirst);
                    setPathState(5);

                }
                break;
            case 5:
                //
                if (!follower.isBusy()){

                    follower.followPath(scorePickup);
                    setPathState(6);

                }
                break;
            case 6:
                if (!follower.isBusy()){

                    follower.followPath(scoreDropoff);
                    if (scoreCount<3){
                        scoreCount ++;
                        setPathState(5);

                    }else{
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if (!follower.isBusy()){

                    follower.followPath(park);
                    setPathState(-1);
                }
        }
    }
    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop(){}

    @Override
    public void start(){
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop(){
    }
}
