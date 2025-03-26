package Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
@TeleOp
public class PIDTuning extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public DcMotorEx S1Motor, S2Motor, AMotor, FL, FR, BR, BL;

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
    }

    @Override
    public void runOpMode(){
        initHardware();

        waitForStart();

        while(opModeIsActive()){

            armTarget = Math.max(10, Math.min(1000, armTarget));
            slideTarget = Math.max(200, Math.min(3500, slideTarget));

            armPower = armPIDF(armTarget, AMotor);
            AMotor.setPower(armPower);

            slidePower = slidePIDF(slideTarget, S1Motor,S2Motor);
            S1Motor.setPower(slidePower);
            S2Motor.setPower(slidePower);
            dashboardTelemetry.addData("slideTarget",slideTarget);
            dashboardTelemetry.addData("s1",S1Motor.getCurrentPosition());
            dashboardTelemetry.addData("s2",S2Motor.getCurrentPosition());
            dashboardTelemetry.addData("armTarget",armTarget);
            dashboardTelemetry.addData("arm",AMotor.getCurrentPosition());
            telemetry.addData("armTarget", armTarget);
            telemetry.addData("armPower", armPower);
            telemetry.addData("slideTarget", slideTarget);
            telemetry.addData("slidePower", slidePower);
            dashboardTelemetry.update();
            telemetry.update();
        }
    }

    public double armPIDF(double target, DcMotorEx motor){
        armPIDF.setPIDF(armP,armI,armD,armF);
        int currentPosition = motor.getCurrentPosition();
        double output = armPIDF.calculate(currentPosition, target);

        return output;
    }

    public double slidePIDF(double target, DcMotorEx motor,DcMotorEx motor2){
        slidePIDF.setPIDF(slideP,slideI,slideD,slideF);
        int currentPosition = (motor.getCurrentPosition()+motor2.getCurrentPosition())/2;
        double output = slidePIDF.calculate(currentPosition, target);

        return output;
    }
}