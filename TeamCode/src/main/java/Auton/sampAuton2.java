package Auton;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

import Teleop.sample;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Sample 2", group = "Auto")
public class sampAuton2 extends OpMode {
    private DcMotorEx AMotor,S1Motor,S2Motor;
    private Servo wrist,claw,rotation;
    private Limelight3A limelight;
    private Follower follower;
    private Timer opmodeTimer,loopTimer,stateTimer;

    public double wristPar = 0.1, wristPerp = 0.62, wristOuttake = 0.85;
    public double clawOpen =  0.43, clawClose = 0.74;
    public double rotationPos = 0.46;
    public double armDown = 25;
    public double armPar = 100, armUp = 890, slidePar = 100,slideUp = 1400;
    public int slideInterval = 15;
    public double outToRestBuffer = 600, restToOuttake = 1000;
    public boolean intaking = false;
    //  ARM PID
    PIDFController armPIDF = new PIDFController(0,0,0, 0);
    double armP = 0.03, armI = 0, armD = 0, armF = 0;
    double armPE = 0.001, armIE = 0, armDE = 0, armFE = 0;
    double armTarget = 0.0;

    //  SLIDES PID
    PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    double slideP = 0.017, slideI = 0, slideD = 0.00018, slideF = 0;
    //    static double slidePE = 0.008, slideIE = 0, slideDE = 0.00018, slideFE = 0;
    double slideTarget = 0.0;
    double slidePower = 0.0;

    LLResult result;
    double [] python;
    double angle = 0.46;
    double tx,ty;

    boolean slow;
    boolean followerDone= false;
    boolean followerDoneDelay = false;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int state;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(17, 126, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(28.5, 120, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(28.5, 130, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(28.5, 130, Math.toRadians(26));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 98, Math.toRadians(270));

    private Pose subPose;

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 115, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park;
    private PathChain scorePreload,grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3,sub1,scoreSub1;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    @Override
    public void init() {
        AMotor = hardwareMap.get(DcMotorEx.class,"AMotor");
        S1Motor = hardwareMap.get(DcMotorEx.class,"S1Motor");
        S2Motor = hardwareMap.get(DcMotorEx.class,"S2Motor");

        wrist = hardwareMap.get(Servo.class,"wrist");
        rotation = hardwareMap.get(Servo.class,"rotation");
        claw = hardwareMap.get(Servo.class,"claw");

        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        stateTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        AMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        S1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        S2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AMotor.setPower(0);

        S1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        S1Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        S1Motor.setPower(0);

        S2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        S2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        S2Motor.setPower(0);

        wrist.setPosition(wristPerp);
        rotation.setPosition(rotationPos);
        claw.setPosition(clawClose);
        limelight.pipelineSwitch(1);
        limelight.stop();

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose),   new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose),   new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

    }

    public void next(){
        state += 1;
        stateTimer.resetTimer();
    }

    //ACTIONS
    public boolean ArmRestToScore(){
        slow = false;
        armTarget = 890;
        if (AMotor.getCurrentPosition()>700){
            RotationScore();
            slideTarget = 1450;
        }
        if (S1Motor.getCurrentPosition()>1400){
            WristOuttaking();
            return true;
        }else{
            WristPar();
        }
        return false;
    }

    public boolean ArmScoreToRest(){
        RotationNormal();
        WristPar();
        slideTarget = 300;
        if (S1Motor.getCurrentPosition()<400){
            slow = true;
            armTarget = 250;
        }
        return AMotor.getCurrentPosition() < 270;
    }

    public boolean ArmRestThird(){
        RotationNormal();
        WristPar();
        slideTarget = 100;
        if (S1Motor.getCurrentPosition()<120){
            slow = true;
            armTarget = 250;
        }
        return AMotor.getCurrentPosition()<270;
    }

    public boolean ArmDownToGrab(){
        slow = true;
        armTarget = 15;
        return AMotor.getCurrentPosition() < 20;
    }
    public boolean ArmRest(){
        slow = true;
        armTarget = 200;
        return AMotor.getCurrentPosition()>180;
    }

    public boolean SlideThird(){
        slideTarget = 325;
        return S1Motor.getCurrentPosition()<335;
    }



    public void WristOuttaking(){
        wrist.setPosition(wristOuttake);
    }
    public void WristPar(){
        wrist.setPosition(wristPar);
    }
    public void WristPerp(){
        wrist.setPosition(wristPerp);
    }

    public void ClawClose(){
        claw.setPosition(clawClose);
    }
    public void ClawOpen(){
        claw.setPosition(clawOpen);
    }

    public void RotationSpecial(){
        rotation.setPosition(.75);
    }
    public void RotationNormal(){
        rotation.setPosition(rotationPos);
    }
    public void RotationScore() {rotation.setPosition(1);}
    public void RotationSub(){
        rotation.setPosition(angle);
    }

    public boolean LimelightOpen(){
        limelight.start();
        WristPar();
        result = limelight.getLatestResult();
        if (result!= null){
            python = result.getPythonOutput();
            telemetry.addData("result", Arrays.toString(python));
            double tx = python[6];
            double ty = python[5]-8;
            double rawAngle = python[4];
            angle = (rawAngle >= 0) ? (1 - rawAngle / 180) : (-rawAngle / 180);
            slideTarget = 230 + ty*2.2;
            subPose = new Pose(follower.getPose().getX() + 6.7 - tx/4.2,follower.getPose().getY(),follower.getPose().getHeading()); // 6.7 - tx/4.5
            sub1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(parkPose),new Point(subPose)))
                    .setLinearHeadingInterpolation(parkPose.getHeading(),parkPose.getHeading())
                    .build();
            scoreSub1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(subPose),new Point(scorePose)))
                    .setLinearHeadingInterpolation(subPose.getHeading(), scorePose.getHeading())
                    .build();
            if (S1Motor.getCurrentPosition()>slideTarget-20) {
                limelight.pause();
                return true;
            }

        }return false;

    }
    public void LimelightClose(){limelight.stop();}



    public void autonomousPathUpdate() {
        switch (state) {
//            case 0:
//                if (!followerDone){
//                    follower.followPath(scorePreload,true);
//                    followerDone = true;
//                }
//                if (ArmRestToScore() && follower.getPose().getY()>123){
//                    followerDone = false;
//                    next();
//                }
//                break;
//            case 1:
//            case 5:
//            case 9:
//                if (stateTimer.getElapsedTimeSeconds()>.2){
//                    ClawOpen();
//                    if (stateTimer.getElapsedTimeSeconds()>.5){
//                        next();
//                    }
//                }break;
//            case 2:
//                if (!followerDone){
//                    follower.followPath(grabPickup1,true);
//                    followerDone = true;
//                }
//                if (ArmScoreToRest() && follower.getPose().getX()>25){
//                    followerDone = false;
//                    next();
//
//                }break;
//            case 3:
//            case 7:
//            case 11:
//                if (stateTimer.getElapsedTimeSeconds()>.5){
//                    if (ArmDownToGrab()){
//                        next();
//                    }
//                }break;
//            case 4:
//                if (stateTimer.getElapsedTimeSeconds()>.4){
//                    ClawClose();
//                    if (stateTimer.getElapsedTime()>.7){
//                        if (!followerDone){
//                            follower.followPath(scorePickup1);
//                            followerDone = true;
//                        }
//                        if (ArmRestToScore() && follower.getPose().getX()<20){
//                            followerDone = false;
//                            next();
//                        }
//                    }
//                }break;
//            case 6:
//                if (!followerDone){
//                    follower.followPath(grabPickup2);
//                    followerDone = true;
//                }
//                if (ArmScoreToRest() && follower.getPose().getX()>25){
//                    followerDone = false;
//                    next();
//                }
//                break;
//            case 8:
//                if (stateTimer.getElapsedTimeSeconds()>.2){
//                    ClawClose();
//                    if (stateTimer.getElapsedTime()>.7){
//                        if (!followerDone){
//                            follower.followPath(scorePickup2);
//                            followerDone = true;
//                        }
//                        if (ArmRestToScore() && follower.getPose().getX()<20){
//                            followerDone = false;
//                            next();
//                        }
//                    }
//                }break;
//            case 10:
//                if (!followerDone){
//                    follower.followPath(grabPickup3);
//                    followerDone = true;
//                }
//                if (ArmScoreToRest() && follower.getPose().getX()>27){
//                    RotationSpecial();
//                    followerDone = false;
//                    next();
//                }
//                break;
//            case 12:
//                if (stateTimer.getElapsedTimeSeconds()>.2){
//                    ClawClose();
//                    if (stateTimer.getElapsedTime()>.7){
//                        if (!followerDone){
//                            follower.followPath(scorePickup3);
//                            followerDone = true;
//                        }
//                        if (ArmRestToScore() && follower.getPose().getX()<20){
//                            followerDone = false;
//                            next();
//                        }
//                    }
//                }break;
//
            case 0:
                if (!followerDone) {
                    follower.followPath(scorePreload, true);
                    followerDone = true;
                }
                if (ArmRestToScore() && follower.getPose().getY() > 123) {
                    if (!followerDoneDelay) {
                        followerDoneDelay = true;
                        stateTimer.resetTimer();
                    } else if (stateTimer.getElapsedTimeSeconds() > 0.5) {
                        followerDone = false;
                        followerDoneDelay = false;
                        next();
                    }
                } else {
                    followerDoneDelay = false;
                }
                break;

            case 1:
            case 6:
            case 11:
            case 17:
            case 25:
                if (stateTimer.getElapsedTimeSeconds() > 0.2) {
                    ClawOpen();
                    if (stateTimer.getElapsedTimeSeconds() > 0.4) {
                        WristPar();
                        if (stateTimer.getElapsedTimeSeconds()>0.5){
                            next();
                        }

                    }
                }
                break;

            case 2:
                if (!followerDone) {
                    follower.followPath(grabPickup1, true);
                    followerDone = true;
                }
                if (ArmScoreToRest() && follower.headingError < 0.02) {
                    if (!followerDoneDelay) {
                        followerDoneDelay = true;
                        stateTimer.resetTimer();
                    } else if (stateTimer.getElapsedTimeSeconds() > 0.5) {
                        followerDone = false;
                        followerDoneDelay = false;
                        next();
                    }
                } else {
                    followerDoneDelay = false;
                }
                break;

            case 3:
            case 8:
            case 14:
                if (stateTimer.getElapsedTime() > .7) {
                    if (ArmDownToGrab()) {
                        next();
                    }
                }
                break;

            case 4:
            case 9:
            case 15:
                if (stateTimer.getElapsedTimeSeconds() > 0.3) {
                    ClawClose();
                    next();
                }
                break;

            case 5:
                if (stateTimer.getElapsedTimeSeconds() > 0.3) {
                    if (!followerDone) {
                        follower.setMaxPower(.6);
                        follower.followPath(scorePickup1, true);
                        followerDone = true;
                    }
                    if (ArmRestToScore() && follower.getPose().getX() < 20) {
                        if (!followerDoneDelay) {
                            followerDoneDelay = true;
                            stateTimer.resetTimer();
                        } else if (stateTimer.getElapsedTimeSeconds() > 0.5) {
                            followerDone = false;
                            followerDoneDelay = false;
                            next();
                        }
                    } else {
                        followerDoneDelay = false;
                    }
                }
                break;

            case 7:
                if (!followerDone) {
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup2, true);
                    followerDone = true;
                }
                if (ArmScoreToRest() && follower.headingError < 0.025) {
                    if (!followerDoneDelay) {
                        followerDoneDelay = true;
                        stateTimer.resetTimer();
                    } else if (stateTimer.getElapsedTimeSeconds() > 0.5) {
                        followerDone = false;
                        followerDoneDelay = false;
                        next();
                    }
                } else {
                    followerDoneDelay = false;
                }
                break;

            case 10:
                if (stateTimer.getElapsedTimeSeconds() > 0.3) {
                    if (!followerDone) {
                        follower.setMaxPower(.6);
                        follower.followPath(scorePickup2, true);
                        followerDone = true;
                    }
                    if (ArmRestToScore() && follower.getPose().getX() < 20) {
                        if (!followerDoneDelay) {
                            followerDoneDelay = true;
                            stateTimer.resetTimer();
                        } else if (stateTimer.getElapsedTimeSeconds() > 0.5) {
                            followerDone = false;
                            followerDoneDelay = false;
                            next();
                        }
                    } else {
                        followerDoneDelay = false;
                    }
                }
                break;

            case 12:
                if (!followerDone) {
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup3, true);
                    followerDone = true;
                }
                if (ArmRestThird() && follower.headingError < 0.02) {
                    RotationSpecial();
                    if (!followerDoneDelay) {
                        followerDoneDelay = true;
                        stateTimer.resetTimer();
                    } else if (stateTimer.getElapsedTimeSeconds() > 0.5) {
                        followerDone = false;
                        followerDoneDelay = false;
                        next();
                    }
                } else {
                    followerDoneDelay = false;
                }
                break;
            case 13:
                if (SlideThird()){
                    next();
                }break;
            case 16:
                if (stateTimer.getElapsedTimeSeconds() > 0.3) {
                    if (!followerDone) {
                        follower.setMaxPower(.6);
                        follower.followPath(scorePickup3, true);
                        followerDone = true;
                    }
                    if (ArmRestToScore() && follower.getPose().getX() < 20) {
                        if (!followerDoneDelay) {
                            followerDoneDelay = true;
                            stateTimer.resetTimer();
                        } else if (stateTimer.getElapsedTimeSeconds() > 0.5) {
                            followerDone = false;
                            followerDoneDelay = false;
                            next();
                        }
                    } else {
                        followerDoneDelay = false;
                    }
                }
                break;

            case 18:
                if (follower.getPose().getY()<103){
                    follower.setMaxPower(.5);
                }else{
                    follower.setMaxPower(1);
                }
                if (!followerDone) {
                    follower.followPath(park, true);
                    followerDone = true;
                }
                if (ArmScoreToRest() && follower.getPose().getY() < 101) {
                    if (!followerDoneDelay) {
                        followerDoneDelay = true;
                        stateTimer.resetTimer();
                    } else if (stateTimer.getElapsedTimeSeconds() > 1) {
                        followerDone = false;
                        followerDoneDelay = false;
                        next();
                    }
                } else {
                    followerDoneDelay = false;
                }
                break;
            case 19:
                if (LimelightOpen()){
                    next();
                }break;
            case 20:
                if (!followerDone){
                    RotationSub();
                    follower.followPath(sub1);
                }
                if (follower.driveError<0.01){
                    if (!followerDoneDelay){
                        followerDoneDelay = true;
                        stateTimer.resetTimer();
                    }
                    else if (stateTimer.getElapsedTimeSeconds()>1){
                        followerDone = false;
                        followerDoneDelay = false;
                        next();
                    }
                }else {
                    followerDoneDelay = false;
                }break;
            case 21:
                if (stateTimer.getElapsedTimeSeconds()>1){
                    if (ArmDownToGrab()){
                        next();
                    }
                }break;
            case 22:
                if (stateTimer.getElapsedTimeSeconds()>.2){
                    ClawClose();
                    next();
                }break;
            case 23:
                if (stateTimer.getElapsedTimeSeconds()>0.3) {
                    if (ArmRest()) {
                        next();
                    }
                }break;
            case 24:
                if (follower.getPose().getX() < 30) {
                    follower.setMaxPower(.6);
                } else {
                    follower.setMaxPower(1);
                }
                if (!followerDone) {
                    follower.followPath(scoreSub1, true);
                    followerDone = true;
                }
                if (ArmRestToScore() && follower.getPose().getX() < 20) {
                    if (!followerDoneDelay) {
                        followerDoneDelay = true;
                        stateTimer.resetTimer();
                    } else if (stateTimer.getElapsedTimeSeconds() > 0.5) {
                        followerDone = false;
                        followerDoneDelay = false;
                        next();
                    }
                } else {
                    followerDoneDelay = false;
                }
                break;








        }
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.drawOnDashBoard();
        follower.update();
        autonomousPathUpdate();
        AMotor.setPower(armPIDF(armTarget,slow));
        S1Motor.setPower(slidePIDF(slideTarget));
        S2Motor.setPower(slidePIDF(slideTarget));
        // Feedback to Driver Hub
        telemetry.addData("state",state);
        telemetry.addData("busy",follower.isBusy());
        telemetry.addData("result",Arrays.toString(python));
        telemetry.addData("arm",AMotor.getCurrentPosition());
        telemetry.addData("slide",S1Motor.getCurrentPosition());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("hError",follower.headingError);
        telemetry.update();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        WristPerp();
        ClawClose();
        RotationNormal();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        state = 0;

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    public double armPIDF(double target, boolean slow){
        if (slow){
            armPIDF.setPIDF(armPE,armIE,armDE,armFE);
        }else {
            armPIDF.setPIDF(armP, armI, armD, armF);
        }
        int currentPosition = AMotor.getCurrentPosition();
        double output = armPIDF.calculate(currentPosition, target);

        return output;
    }

    public double slidePIDF(double target){
        slidePIDF.setPIDF(slideP, slideI, slideD, slideF);
        int currentPosition = (S1Motor.getCurrentPosition()+S2Motor.getCurrentPosition())/2;
        double output = slidePIDF.calculate(currentPosition, target);


        return output;
    }
}

