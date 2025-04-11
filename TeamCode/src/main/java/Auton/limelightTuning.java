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

@Autonomous(name = "Limelight", group = "Auto")
public class limelightTuning extends OpMode {
    private DcMotorEx AMotor,S1Motor,S2Motor;
    private Servo wrist,claw,rotation;
    private Limelight3A limelight;
    private Follower follower;
    private Timer opmodeTimer,loopTimer,stateTimer;

    public double wristPar = 0.1, wristPerp = 0.62, wristOuttake = 0.82;
    public double clawOpen =  0.3, clawClose = 0.74;
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
    double angle = 0.46;
    double tx = 0,ty = 0;

    boolean slow;
    boolean followerDone= false;

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
    private final Pose scorePose = new Pose(19, 124, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(26, 121, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(26, 131, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(28, 133, Math.toRadians(30));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 98, Math.toRadians(270));

    private Pose subPose = new Pose(60+tx,98+ty,Math.toRadians(270));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 115, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park;
    private PathChain scorePreload,grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3,sub1;

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
            slideTarget = 1400;
        }
        if (S1Motor.getCurrentPosition()>1300){
            WristOuttaking();
            return true;
        }else{
            WristPar();
        }
        return false;
    }

    public boolean ArmScoreToRest(){
        WristPar();
        slideTarget = 300;
        if (S1Motor.getCurrentPosition()<500){
            slow = true;
            armTarget = 250;
        }
        return AMotor.getCurrentPosition() < 300;
    }

    public boolean ArmDownToGrab(){
        slow = true;
        armTarget = 25;
        return AMotor.getCurrentPosition() < 30;
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
    public void RotationSub(){
        rotation.setPosition(angle);
    }

    public boolean LimelightOpen(){
        limelight.start();
        result = limelight.getLatestResult();
        if (result!= null){
            double[] python = result.getPythonOutput();
            double tx = result.getTx();
            double ty = 2*result.getTy();
            double rawAngle = python[4];
            angle = (rawAngle >= 0) ? (1 - rawAngle / 180) : (-rawAngle / 180);
            slideTarget = 25*ty;
            subPose = new Pose(follower.getPose().getX()-2*tx,follower.getPose().getY(),follower.getPose().getHeading());
            sub1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(parkPose),new Point(subPose)))
                    .setLinearHeadingInterpolation(parkPose.getHeading(),parkPose.getHeading())
                    .build();
            return true;
        }else{
            return false;
        }

    }
    public void LimelightClose(){limelight.stop();}



    public void autonomousPathUpdate() {
        switch (state) {
            case 0:
                ClawOpen();
                ArmScoreToRest();
                if (LimelightOpen()){
                    next();
                }break;
            case 1:
                if (!followerDone){
                    follower.followPath(sub1);
                }
                if (follower.driveError<.5){
                    followerDone = false;
                    next();
                }
            case 2:
                if (stateTimer.getElapsedTimeSeconds()>.5){
                    RotationSub();
                    if (ArmDownToGrab()){
                        next();
                    }
                }break;
            case 3:
                if (stateTimer.getElapsedTimeSeconds()>.5){
                    ClawClose();
                    next();
                }break;
            case 4:
                if (stateTimer.getElapsedTimeSeconds()>.5){
                    ArmScoreToRest();
                }

        }
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        if (gamepad1.dpad_down){
            slideTarget -= 5;
        }else if (gamepad1.dpad_up){
            slideTarget += 5;
        }
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
        telemetry.addData("arm",AMotor.getCurrentPosition());
        telemetry.addData("slide",S1Motor.getCurrentPosition());
        telemetry.addData("tx",tx);
        telemetry.addData("ty",ty);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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

