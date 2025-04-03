package Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
@TeleOp
public class limelight extends LinearOpMode { ;

    public DcMotorEx S1Motor, S2Motor, AMotor, FL, FR, BR, BL;
    public Servo wrist,rotation;
    public Limelight3A limelight;
    public LLResult result;
    public double clawROpen = 0.25, clawRClose = 0.75;
    PIDFController armPIDF = new PIDFController(0,0,0, 0);
    public static double armP = 0.02, armI = 0, armD = 0.001, armF = 0;
    //    extended PID
    public static double armTarget = 0.0;
    public double armPower = 0.0;
    //  SLIDES PID
    PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    public static double slideP = 0.017, slideI = 0, slideD = 0.00018, slideF = 0;
    public static double slideTarget = 0.0;
    public double slidePower = 0.0;
    // slide down -> p: 0.045, d: 0.00019, max: 2000
    //slide extended -> p:0.045, d:0.00026, max:3200
    // arm down -> p: 0.02, d: 0.00022, min 250? max 1300

    public enum Mode {
        INTAKING,
        OUTTAKING,
        REST,
        HANG
    }

    Mode mode = Mode.REST;


    public void initHardware() {
//      DRIVE MOTORS

        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        wrist = hardwareMap.get(Servo.class,"wrist");
        rotation = hardwareMap.get(Servo.class,"rotation");

        FL.setDirection(DcMotorEx.Direction.FORWARD);
        BL.setDirection(DcMotorEx.Direction.FORWARD);
        FR.setDirection(DcMotorEx.Direction.REVERSE);
        BR.setDirection(DcMotorEx.Direction.REVERSE);


        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);


//        ARM AND SLIDE
        S1Motor = hardwareMap.get(DcMotorEx.class, "S1Motor");
        S2Motor = hardwareMap.get(DcMotorEx.class, "S2Motor");
        AMotor = hardwareMap.get(DcMotorEx.class, "AMotor");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");


        S1Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        S2Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        AMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        S1Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        S2Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        AMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        S1Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        S2Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        AMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        S1Motor.setPower(0);
        S2Motor.setPower(0);
        AMotor.setPower(0);
        limelight.pipelineSwitch(1);


    }

    @Override
    public void runOpMode() {
        initHardware();

        waitForStart();
        while (opModeIsActive()) {
            wrist.setPosition(0.62);
            if (gamepad1.x) {
                limelight.start();
                result = limelight.getLatestResult();
                if (result != null) {
                    double[] pythonOutputs = result.getPythonOutput();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("py angle", pythonOutputs[4]);
                    double angle = pythonOutputs[4];
                    if (angle<0) {angle=angle/180;} else {angle = (180+angle)/180;}
                    telemetry.addData("angle",angle);
                    rotation.setPosition(angle);
                    limelight.pause();
                }
                telemetry.update();
            }
        }

    }
    }