package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "TeleOp (test)")
public class DrivePrototyping extends LinearOpMode {
    // private TouchSensor Touch;
    private DcMotorEx MotorFL; // this is the motor pluged into 0
    private DcMotorEx MotorFR; // this is the motor pluged into 1
    private DcMotorEx MotorBL; // this is the motor pluged into 2
    private DcMotorEx MotorBR; // this is the motor pluged into 3
    private DcMotorEx ArmL;
    private DcMotorEx ArmR;
    private DcMotorEx PullDownL;
    private DcMotorEx PullDownR;
    private Servo ClawArm;
    private Servo ClawHand1;
    private Servo ClawHand2;
    private Servo servoTest;
    private Servo PlaneAngle;
    private BNO055IMU imu;
    private ColorSensor Color;
    // private Servo ServoArm;
    // private Servo ServoClaw;
    // private DcMotor Arm1;
    // private DcMotor Arm2;
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        MotorFL = hardwareMap.get(DcMotorEx.class, "MotorFL"); // this is the motor pluged into 0
        MotorFR = hardwareMap.get(DcMotorEx.class, "MotorFR"); // this is the motor pluged into 1
        MotorBL = hardwareMap.get(DcMotorEx.class, "MotorBL"); // this is the motor pluged into 2
        MotorBR = hardwareMap.get(DcMotorEx.class, "MotorBR"); // this is the motor pluged into 3
        ArmL = hardwareMap.get(DcMotorEx.class, "ArmL");
        ArmR = hardwareMap.get(DcMotorEx.class, "ArmR");
        PullDownL = hardwareMap.get(DcMotorEx.class, "PullDownL");
        PullDownR = hardwareMap.get(DcMotorEx.class, "PullDownR");
        ClawArm = hardwareMap.get(Servo.class, "ClawArm");
        ClawHand1 = hardwareMap.get(Servo.class, "ClawHand1");
        ClawHand2 = hardwareMap.get(Servo.class, "ClawHand2");
        servoTest = hardwareMap.get(Servo.class, "ServoTest");
        PlaneAngle = hardwareMap.get(Servo.class, "PlaneAngle");
        Color = hardwareMap.get(ColorSensor.class, "col");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");

        MotorFL.setDirection(DcMotorEx.Direction.FORWARD);
        MotorFR.setDirection(DcMotorEx.Direction.REVERSE);
        MotorBL.setDirection(DcMotorEx.Direction.REVERSE);
        MotorBR.setDirection(DcMotorEx.Direction.REVERSE);

//        MotorFL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        MotorFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        MotorBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        MotorBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        ArmL.setDirection(DcMotorEx.Direction.FORWARD);
        ArmR.setDirection(DcMotorEx.Direction.FORWARD);
        PullDownL.setDirection(DcMotorEx.Direction.FORWARD);
        PullDownR.setDirection(DcMotorEx.Direction.FORWARD);
        ArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PullDownL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PullDownR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double MotorForwards = 0;
        double MotorSideways = 0;

        boolean ModeToggle = false;

        double rightTrigger;
        double leftTrigger;

        boolean validStick = false;

        double slowFactor;

        telemetry.update();
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        // PlaneAngle.setPosition(0.42);
        while (opModeIsActive()) {
            boolean ButtonX1 = gamepad1.x;
            boolean ButtonY1 = gamepad1.y;
            boolean ButtonA1 = gamepad1.a;
            boolean ButtonB1 = gamepad1.b;
            boolean ButtonDPup1 = gamepad1.dpad_up;
            boolean ButtonDPdown1 = gamepad1.dpad_down;
            boolean ButtonDPleft1 = gamepad1.dpad_left;
            boolean ButtonDPright1 = gamepad1.dpad_right;
            boolean ButtonRBump1 = gamepad1.right_bumper;
            boolean ButtonLBump1 = gamepad1.left_bumper;
            boolean ButtonOptions1 = gamepad1.options;
            boolean ButtonShare1 = gamepad1.share;
            float TriggerR21 = gamepad1.right_trigger;
            float TriggerL21 = gamepad1.left_trigger;

            // input for controller number 2
            boolean ButtonX2 = gamepad2.x;
            boolean ButtonY2 = gamepad2.y;
            boolean ButtonA2 = gamepad2.a;
            boolean ButtonB2 = gamepad2.b;
            boolean ButtonDPup2 = gamepad2.dpad_up;
            boolean ButtonDPdown2 = gamepad2.dpad_down;
            boolean ButtonDPleft2 = gamepad2.dpad_left;
            boolean ButtonDPright2 = gamepad2.dpad_right;
            boolean ButtonRBump2 = gamepad2.right_bumper;
            boolean ButtonLBump2 = gamepad2.left_bumper;
            boolean ButtonOptions2 = gamepad2.options;
            boolean ButtonShare2 = gamepad2.share;
            float TriggerR22 = gamepad2.right_trigger;
            float TriggerL22 = gamepad2.left_trigger;

            // define the encoders for the motors
            int MotorFLEncoder = MotorFL.getCurrentPosition();
            int MotorFREncoder = MotorFR.getCurrentPosition();
            int MotorBLEncoder = MotorBL.getCurrentPosition();
            int MotorBREncoder = MotorBR.getCurrentPosition();

            // input for controller 1
            MotorForwards = 2500 * Math.pow(-this.gamepad1.left_stick_y, 3);
            MotorSideways = 2500 * Math.pow(-this.gamepad1.left_stick_x, 3);

            rightTrigger = 1550 * Math.pow(-this.gamepad1.right_trigger, 3);
            leftTrigger = 1550 * Math.pow(-this.gamepad1.left_trigger, 3);

            if (gamepad1.right_bumper) {
                rightTrigger = -800;
            }

            if (gamepad1.left_bumper) {
                leftTrigger = -800;
            }

            if (Math.abs(rightTrigger) < 200) {
                rightTrigger = 0;
            }

            if (Math.abs(leftTrigger) < 200) {
                leftTrigger = 0;
            }

            if (Math.abs(MotorForwards) < 200) {
                MotorForwards = 0;
            }

            if (Math.abs(MotorSideways) < 200) {
                MotorSideways = 0;
            }


            // get IMU and clamp to 0-360
            double angle = imu.getAngularOrientation().firstAngle;
            if (angle < 0) {
                angle += 360;
            }

            //adjust motor powers
            double angleInRadians  = Math.toRadians(angle);
            double newForward = MotorForwards * Math.cos(angleInRadians) +
                    MotorSideways * Math.sin(angleInRadians);
            MotorSideways = -MotorForwards * Math.sin(angleInRadians) +
                    MotorSideways * Math.cos(angleInRadians);
            MotorForwards = newForward;

            if (ButtonY1) {
                ModeToggle = false;
            }

            if (ButtonX1) {
                ModeToggle = true;
            }

            telemetry.addData("ModeToggle: ", ModeToggle);
            if (ModeToggle) {
                MotorForwards = MotorForwards / 2;
                MotorSideways = MotorSideways / 2;
            }

            // set the motor powers
            MotorFL.setVelocity(MotorForwards - MotorSideways - rightTrigger + leftTrigger);
            MotorFR.setVelocity(MotorForwards + MotorSideways + rightTrigger - leftTrigger);
            MotorBL.setVelocity(MotorForwards + MotorSideways - rightTrigger + leftTrigger);
            MotorBR.setVelocity(MotorForwards - MotorSideways + rightTrigger - leftTrigger);

            telemetry.addData("Forwards: ", MotorForwards);
            telemetry.addData("Sideways: ", MotorSideways);
            telemetry.addData("Angle", angle);
            telemetry.addData("Status ButtonX1: ", ButtonX1);
            telemetry.addData("Status rightTrigger1: ", TriggerR21);
            telemetry.addData("Status Mode1: ", ButtonOptions2);
            telemetry.addData("Status Share1: ", ButtonShare2);
            telemetry.addData("MotorFL encoder: ", MotorFLEncoder);
            telemetry.addData("MotorFR encoder: ", MotorFREncoder);
            telemetry.addData("MotorBL encoder: ", MotorBLEncoder);
            telemetry.addData("MotorBR encoder: ", MotorBREncoder);
            telemetry.addData("Color Red:", Color.red());
            telemetry.addData("Color Blue:", Color.blue());
            telemetry.addData("Color Green:", Color.green());
            telemetry.update();


            if (ButtonDPright2) { // raise the arm
                ClawArm.setPosition(0.15);
            }

            if (ButtonDPleft2) { // lower the arm
                ClawArm.setPosition(0.5);
            }

            if (ButtonDPup2) {
                ClawArm.setPosition(0);
            }

            if (ButtonDPdown2 || ButtonDPdown1) {
                ClawArm.setPosition(0.32);
            }

            // open claw
            if (ButtonX2) {
                ClawHand1.setPosition(0.5);
                ClawHand2.setPosition(0.5);
            }

            // close claw
            if (ButtonA2) {
                ClawHand1.setPosition(0.1);
                ClawHand2.setPosition(0.1);
            }

            // left claw up
            if (-gamepad2.left_stick_y > .5) {
                ClawHand1.setPosition(0.5);
            }
            // left claw down
            if (-gamepad2.left_stick_y < -.5) {
                ClawHand1.setPosition(0.1);
            }

            // right claw up
            if (-gamepad2.right_stick_y > .5) {
                ClawHand2.setPosition(0.5);
            }
            // right claw down
            if (-gamepad2.right_stick_y < -.5) {
                ClawHand2.setPosition(0.1);
            }

            if (ButtonRBump2) { // pulling that arm(linear slides) up!
                ArmL.setVelocity(-2500);
                ArmR.setVelocity(2500);
                PullDownL.setVelocity(-2500);
                PullDownR.setVelocity(2500);
            } else if (ButtonLBump2) {
                ArmR.setVelocity(-2500);
                ArmL.setVelocity(2500);
                PullDownR.setVelocity(-2500);
                PullDownL.setVelocity(2500);
            } else {
                ArmR.setVelocity(0);
                ArmL.setVelocity(0);
                PullDownR.setVelocity(0);
                PullDownL.setVelocity(0);
            }

            if (ButtonDPup2) {
                PullDownL.setPower(1);
                PullDownR.setPower(-1);
            }
        }
    }
}
