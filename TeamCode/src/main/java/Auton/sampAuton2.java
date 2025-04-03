package Auton;

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

@Autonomous(name = "Sample", group = "Autonomous")
public class sampAuton2 extends OpMode {
    private DcMotorEx AMotor,S1Motor,S2Motor;
    private Servo wrist,claw,rotation;
    private Limelight3A limelight;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

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
    static double armP = 0.03, armI = 0, armD = 0, armF = 0;
    static double armPE = 0.01, armIE = 0, armDE = 0, armFE = 0.005;
    static double armTarget = 0.0;

    //  SLIDES PID
    PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    static double slideP = 0.017, slideI = 0, slideD = 0.00018, slideF = 0;
    //    static double slidePE = 0.008, slideIE = 0, slideDE = 0.00018, slideFE = 0;
    static double slideTarget = 0.0;
    double slidePower = 0.0;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState,actionState;

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

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 115, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park;
    private PathChain scorePreload,grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

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

    //ACTIONS
    public boolean RestToOuttaking(){
        armTarget = armUp;
        if (AMotor.getCurrentPosition()>700){
            WristPar();
            slideTarget = slideUp;
            if (S1Motor.getCurrentPosition()>1300) {
                WristOuttaking();
                return true;
            }
        }
        return false;
    }

    public boolean OuttakingToRest(){
        WristPar();
        slideTarget = slidePar;
        if (S1Motor.getCurrentPosition()<120){
            armTarget = armPar;
            WristPerp();
            if (AMotor.getCurrentPosition()<100){
                return true;
            }
        }
        return false;
    }

    public boolean SlideOut(){
        slideTarget = 300;
        WristPar();
        intaking = true;
        if (S1Motor.getCurrentPosition()>250){
            return true;
        }
        return false;
    }
    public boolean ArmDown(){
        WristPar();
        armTarget = armDown;
        if (AMotor.getCurrentPosition()<armDown+5){
            return true;
        }
        return false;
    }

    public boolean ToRest(){
        armTarget = armPar;
        if (AMotor.getCurrentPosition()>armPar-10){
            slideTarget = slidePar;
            WristPar();
            if (S1Motor.getCurrentPosition()>slidePar-25){
                return true;
            }
        }
        return false;

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

    public void LimelightOpen(){
        limelight.start();
    }
    public void LimelightClose(){limelight.stop();}



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload,false);
                }
                if (actionState >= 4){
                    setPathState(1);

                }
                break;

            case 1:
                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);

                    if (actionState >= 7) {
                        setPathState(2);
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */

                /* Grab Sample */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                if (!follower.isBusy()){
                    follower.followPath(scorePickup1,true);
                    if (actionState >= 11) {
                        setPathState(3);
                    }
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    if(actionState >= 14) {
                        setPathState(4);
                    }
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    if(actionState >= 18) {
                        setPathState(5);
                    }
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    if (actionState >= 21) {
                        setPathState(6);
                    }
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    if (actionState >= 25) {
                        setPathState(7);
                    }
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(park,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }

        switch (actionState){
            case 0:
                WristPar();
                if (RestToOuttaking()){
                    setActionState(2);

                }break;
            case 2:
                if (actionTimer.getElapsedTimeSeconds()>1){
                    ClawOpen();
                    if (actionTimer.getElapsedTimeSeconds()>2) {
                        setActionState(3);
                    }
                }break;
            case 3:
                WristPar();
                if (OuttakingToRest()){
                    setActionState(4);

                }break;
            case 4:
                setPathState(2);
                if (SlideOut()) {
                    setActionState(5);
                }break;

            case 5:
                if (actionTimer.getElapsedTimeSeconds()>1.7){
                    if (ArmDown()) {
                        setActionState(6);
                    }

                }break;
            case 6:
                if (actionTimer.getElapsedTimeSeconds()>.5) {
                    ClawClose();
                }
                if (actionTimer.getElapsedTimeSeconds()>1) {
                    setActionState(7);

                }break;

            case 7:
                if (ToRest()) {
                    setActionState(8);
                }    break;

            case 8:
                if (RestToOuttaking()) {
                    setActionState(9);
                }    break;

            case 9:
                if (actionTimer.getElapsedTimeSeconds()>1){
                    ClawOpen();
                    if (actionTimer.getElapsedTimeSeconds()>2){
                        setActionState(10);

                    }
                }break;
            case 10:
                WristPar();
                if(OuttakingToRest()){
                    setActionState(11);

                }break;
            case 11:
                if (SlideOut()) {
                    setActionState(12);
                }    break;

            case 12:
                if(actionTimer.getElapsedTimeSeconds()>1.7) {
                    if (ArmDown()) {
                        setActionState(13);

                    }
                }break;
            case 13:
                if (actionTimer.getElapsedTimeSeconds()>.5) {
                    ClawClose();
                }
                if (actionTimer.getElapsedTimeSeconds()>1) {
                    setActionState(14);

                }break;

            case 14:
                if (ToRest()) {
                    setActionState(15);
                }    break;

            case 15:
                if (RestToOuttaking()) {
                    setActionState(16);
                }    break;

            case 16:
                if (actionTimer.getElapsedTimeSeconds()>1){
                    ClawOpen();
                    if (actionTimer.getElapsedTimeSeconds()>2){
                        setActionState(17);

                    }
                }break;
            case 17:
                WristPar();
                if(OuttakingToRest()) {
                    setActionState(18);
                }    break;

            case 18:
                RotationSpecial();
                if (SlideOut()) {
                    setActionState(19);
                }    break;

            case 19:
                if (actionTimer.getElapsedTimeSeconds()>2.5){
                    if (ArmDown()){
                        setActionState(20);

                    }
                }break;
            case 20:
                if (actionTimer.getElapsedTimeSeconds()>.5) {
                    ClawClose();
                }
                if (actionTimer.getElapsedTimeSeconds()>1) {
                    setActionState(21);

                }break;
            case 21:
                RotationNormal();
                if (ToRest()){
                    setActionState(22);
                    break;
                }
            case 22:
                if(RestToOuttaking()) {
                    setActionState(23);
                }    break;

            case 23:
                if(actionTimer.getElapsedTimeSeconds()>1){
                    ClawOpen();
                    if (actionTimer.getElapsedTimeSeconds()>2){
                        setActionState(24);

                    }
                }break;
            case 24:
                WristPar();
                if(OuttakingToRest()){
                    setActionState(99);
                }break;
            case 25:
                if (!follower.isBusy()){
                    SlideOut();
                    LimelightOpen();
                }




        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setActionState(int aState){
        actionState = aState;
        actionTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        AMotor.setPower(armPIDF(armTarget,AMotor));
        S1Motor.setPower(slidePIDF(slideTarget,S1Motor,S2Motor));
        S2Motor.setPower(slidePIDF(slideTarget,S1Motor,S2Motor));
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("action state",actionState);
        telemetry.addData("busy",follower.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        AMotor = hardwareMap.get(DcMotorEx.class,"AMotor");
        S1Motor = hardwareMap.get(DcMotorEx.class,"S1Motor");
        S2Motor = hardwareMap.get(DcMotorEx.class,"S2Motor");

        wrist = hardwareMap.get(Servo.class,"wrist");
        rotation = hardwareMap.get(Servo.class,"rotation");
        claw = hardwareMap.get(Servo.class,"claw");

        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

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



    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        setActionState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    public double armPIDF(double target, DcMotorEx motor){
        if (intaking){
            armPIDF.setPIDF(armPE,armIE,armDE,armFE);
        }else {
            armPIDF.setPIDF(armP, armI, armD, armF);
        }
        int currentPosition = motor.getCurrentPosition();
        double output = armPIDF.calculate(currentPosition, target);

        return output;
    }

    public double slidePIDF(double target, DcMotorEx motor,DcMotorEx motor2){
        slidePIDF.setPIDF(slideP, slideI, slideD, slideF);
        int currentPosition = (motor.getCurrentPosition()+motor2.getCurrentPosition())/2;
        double output = slidePIDF.calculate(currentPosition, target);


        return output;
    }
}

