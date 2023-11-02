package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp")
public class Drive extends LinearOpMode {
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
    private Servo ClawHand;
    private Servo servoTest;
    // private Servo ServoArm;
    // private Servo ServoClaw;
    // private DcMotor Arm1;
    // private DcMotor Arm2;
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        MotorFL = hardwareMap.get(DcMotorEx.class, "MotorFL"); // this is the motor pluged into 0
        MotorFR = hardwareMap.get(DcMotorEx.class, "MotorFR"); // this is the motor pluged into 1
        MotorBL = hardwareMap.get(DcMotorEx.class, "MotorBL"); // this is the motor pluged into 2
        MotorBR = hardwareMap.get(DcMotorEx.class, "MotorBR"); // this is the motor pluged into 3
        ArmL = hardwareMap.get(DcMotorEx.class, "ArmL");
        ArmR = hardwareMap.get(DcMotorEx.class, "ArmR");
        PullDownL = hardwareMap.get(DcMotorEx.class, "PullDownL");
        PullDownR = hardwareMap.get(DcMotorEx.class, "PullDownR");
        ClawArm = hardwareMap.get(Servo.class, "ClawArm");
        ClawHand = hardwareMap.get(Servo.class, "ClawHand");
        servoTest = hardwareMap.get(Servo.class, "ServoTest");

        telemetry.addData("Status", "Initialized");

        MotorFL.setDirection(DcMotorEx.Direction.FORWARD);
        MotorFR.setDirection(DcMotorEx.Direction.FORWARD);
        MotorBL.setDirection(DcMotorEx.Direction.FORWARD);
        MotorBR.setDirection(DcMotorEx.Direction.FORWARD);
        ArmL.setDirection(DcMotorEx.Direction.FORWARD);
        ArmR.setDirection(DcMotorEx.Direction.FORWARD);
        PullDownL.setDirection(DcMotorEx.Direction.FORWARD);
        PullDownR.setDirection(DcMotorEx.Direction.FORWARD);
        ArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PullDownL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PullDownR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double TSpeed = 1;
        double BSpeed = 2;
        double FBSpeed = 1.5;
        double LRSpeed = 1.5;
        double ROSpeed = 1.5;

        double MotorPowerLY1 = 0;
        double MotorPowerLX1 = 0;
        double MotorPowerRX1 = 0;
        double MotorPowerRY1 = 0;

        boolean ModeToggle = false;

        boolean armLock = false;
        int arPos = 0;
        int alPos = 0;
        int pdrPos = 0;
        int pdlPos = 0;

        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            // define the encoders for the motors
            int MotorFLEncoder = MotorFL.getCurrentPosition();
            int MotorFREncoder = MotorFR.getCurrentPosition();
            int MotorBLEncoder = MotorBL.getCurrentPosition();
            int MotorBREncoder = MotorBR.getCurrentPosition();

            // input for controller 1
            MotorPowerLY1 = -this.gamepad1.left_stick_y / FBSpeed * TSpeed;
            MotorPowerLX1 = -this.gamepad1.left_stick_x / LRSpeed * TSpeed;
            MotorPowerRX1 = -this.gamepad1.right_stick_x / ROSpeed * TSpeed;
            MotorPowerRY1 = -this.gamepad1.right_stick_y;

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

            //boolean ButtonTouch = Touch.isPressed();

            MotorPowerRX1 = MotorPowerRX1 * BSpeed;
            MotorPowerLX1 = MotorPowerLX1 * BSpeed;
            MotorPowerLY1 = MotorPowerLY1 * BSpeed;


            telemetry.addData("Status lx1: ", MotorPowerLX1);
            telemetry.addData("Status ly1: ", MotorPowerLY1);
            telemetry.addData("Status rx1: ", MotorPowerRX1);
            telemetry.addData("Status ButtonX1: ", ButtonX1);
            telemetry.addData("Status rightTrigger1: ", TriggerR21);
            telemetry.addData("Status Mode1: ", ButtonOptions2);
            telemetry.addData("Status Share1: ", ButtonShare2);
            telemetry.addData("Status ModeToggle1: ", ModeToggle);
            telemetry.addData("MotorFL encoder: ", MotorFLEncoder);
            telemetry.addData("MotorFR encoder: ", MotorFREncoder);
            telemetry.addData("MotorBL encoder: ", MotorBLEncoder);
            telemetry.addData("MotorBR encoder: ", MotorBREncoder);
            //telemetry.addData("Status Touch", ButtonTouch);
            //telemetry.addData("Status lol", Arm1.getCurrentPosition());
            telemetry.update();

            if (MotorPowerRX1 > 1) { // make sure values don't go over 1
                MotorPowerRX1 = 1;
            }
            if (MotorPowerLX1 > 1) { // make sure values don't go over 1
                MotorPowerLX1 = 1;
            }
            if (MotorPowerLY1 > 1) { // make sure values don't go over 1
                MotorPowerLY1 = 1;
            }

            if (ModeToggle) { // slow the robot movement down
                MotorPowerRX1 = MotorPowerRX1 / 1.5;
                MotorPowerLX1 = MotorPowerLX1 / 1.5;
                MotorPowerLY1 = MotorPowerLY1 / 1.5;
                TriggerR21 = TriggerR21 / 2;
                TriggerL21 = TriggerL21 / 2;
            }

            if (ButtonB1) {
                //ModeToggle = true;
                servoTest.setPosition(0);
                //ArmL.setPower(0.5);
                //ArmR.setPower(0.5);
            }

            if (ButtonY1) {
                //ModeToggle = false;
                servoTest.setPosition(0.42);
                //ArmL.setPower(-0.5);
                //ArmR.setPower(-0.5);
            }

            if (ButtonB2) {
                ClawArm.setPosition(0.15);
            }

            if (ButtonY2) {
                ClawArm.setPosition(0.5);
            }

            if (ButtonA2) {
                ClawHand.setPosition(0);
            }

            if (ButtonX2) {
                ClawHand.setPosition(0.4);
            }
            if (armLock == false) {
                ArmL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                PullDownL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                PullDownR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (ButtonRBump2) { // pulling that arm(linear slides) up!
                    //Arm2.setPower(-1);
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
            } else {
                if (ArmR.getCurrentPosition() < arPos-1000) {
                    ArmR.setVelocity(2500);
                    ArmL.setVelocity(-2500);
                    PullDownR.setVelocity(2500);
                    PullDownL.setVelocity(-2500);
                } else {
                    ArmR.setVelocity(0);
                    ArmL.setVelocity(0);
                    PullDownR.setVelocity(0);
                    PullDownL.setVelocity(0);
                }
            }

            if (ButtonDPdown2 && armLock == false) {
                armLock = true;
                arPos = ArmR.getCurrentPosition();
                alPos = ArmL.getCurrentPosition();
                pdrPos = PullDownR.getCurrentPosition();
                pdlPos = PullDownL.getCurrentPosition();

            }
            if (ButtonDPup2 && armLock == true) {
                armLock = false;
            }

            if (ButtonDPup2) {
                PullDownL.setPower(1);
                PullDownR.setPower(-1);
                //Arm1.setPower(1);
            }
            telemetry.addLine(toString().valueOf(ArmL.getVelocity()));
            telemetry.addLine(toString().valueOf(ArmR.getVelocity()));
            telemetry.addLine(toString().valueOf(arPos));
            telemetry.addLine(toString().valueOf(alPos));
            telemetry.addLine(toString().valueOf(pdrPos));
            telemetry.addLine(toString().valueOf(pdlPos));
            telemetry.addLine(toString().valueOf(armLock));
//            if (ButtonDPdown2) {
//                PullDownL.setPower(-1);
//                PullDownR.setPower(1);
//                //Arm1.setPower(-1);
//            }


            if (ButtonDPleft2) {
                //ervoArm.setPosition(0);
            }

            if (ButtonDPright2) {
                //ServoArm.setPosition(1);

            }

            if (ButtonA2) {
                //ServoClaw.setPosition(1);
            }

            if (ButtonX2) {
                //ServoClaw.setPosition(0);
            }

            // do not change unless you know what you are doing!!!
            if (MotorPowerLY1 > -0) { // move forward
                MotorFL.setPower(MotorPowerLY1);
                MotorFR.setPower(-MotorPowerLY1);
                MotorBL.setPower(-MotorPowerLY1);
                MotorBR.setPower(-MotorPowerLY1);
            }

            // do not change unless you know what you are doing!!!
            if (MotorPowerLY1 < 0) { // move backword
                MotorFL.setPower(MotorPowerLY1);
                MotorFR.setPower(-MotorPowerLY1);
                MotorBL.setPower(-MotorPowerLY1);
                MotorBR.setPower(-MotorPowerLY1);
            }

            // do not change unless you know what you are doing!!!
            if (MotorPowerRX1 > 0) { // move left
                MotorFL.setPower(-MotorPowerRX1);
                MotorFR.setPower(-MotorPowerRX1);
                MotorBL.setPower(-MotorPowerRX1);
                MotorBR.setPower(MotorPowerRX1);
            }

            // do not change unless you know what you are doing!!!
            if (MotorPowerRX1 < -0) { // move right
                MotorFL.setPower(-MotorPowerRX1);
                MotorFR.setPower(-MotorPowerRX1);
                MotorBL.setPower(-MotorPowerRX1);
                MotorBR.setPower(MotorPowerRX1);
            }

            // do not change unless you know what you are doing!!!
            if (TriggerL21 > 0) { // rotate left
                MotorFL.setPower(-TriggerL21);
                MotorFR.setPower(-TriggerL21);
                MotorBL.setPower(TriggerL21);
                MotorBR.setPower(-TriggerL21);
            }

            // do not change unless you know what you are doing!!!
            if (TriggerR21 > 0) { // rotate right
                MotorFL.setPower(TriggerR21);
                MotorFR.setPower(TriggerR21);
                MotorBL.setPower(-TriggerR21);
                MotorBR.setPower(TriggerR21);
            }

            // floor all of the motor movement values
            MotorFL.setPower(0);
            MotorFR.setPower(0);
            MotorBL.setPower(0);
            MotorBR.setPower(0);
            //Arm1.setPower(0);
            //Arm2.setPower(0);
        }
    }
}
