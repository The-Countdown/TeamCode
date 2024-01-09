package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Blue Backboard Auto")
public class BlueBackboardAuto extends LinearOpMode {
    private DistanceSensor LeftDistance;
    private DistanceSensor RightDistance;

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
    private ColorSensor Color;
    private IMU imu;
    ElapsedTime runtime = new ElapsedTime();

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    public void zeroMotors() {
        MotorFL.setPower(0);
        MotorFR.setPower(0);
        MotorBL.setPower(0);
        MotorBR.setPower(0);
        MotorFL.setVelocity(0);
        MotorFR.setVelocity(0);
        MotorBL.setVelocity(0);
        MotorBR.setVelocity(0);
    }

    public void resetEncoders() {
        MotorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runOpMode() {
        LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
        Rev2mDistanceSensor sensorTimeOfFlightLeft = (Rev2mDistanceSensor)LeftDistance;
        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
        Rev2mDistanceSensor sensorTimeOfFlightRight = (Rev2mDistanceSensor)RightDistance;
        Color = hardwareMap.get(ColorSensor.class, "col");


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

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Status", "Initialized");
        // max for the distance sensor is about 1000mm (minimal testing)
        // undefined distance for the sensor is 8190mm
        telemetry.addData("deviceName", LeftDistance.getDeviceName() );
        telemetry.addData("LeftRange", String.format("%.01f mm", LeftDistance.getDistance(DistanceUnit.MM)));
        telemetry.addData("LeftRange", String.format("%.01f cm", LeftDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("LeftRange", String.format("%.01f m", LeftDistance.getDistance(DistanceUnit.METER)));
        telemetry.addData("LeftRange", String.format("%.01f in", LeftDistance.getDistance(DistanceUnit.INCH)));
        telemetry.addData("deviceName", RightDistance.getDeviceName() );
        telemetry.addData("RightRange", String.format("%.01f mm", RightDistance.getDistance(DistanceUnit.MM)));
        telemetry.addData("RightRange", String.format("%.01f cm", RightDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("RightRange", String.format("%.01f m", RightDistance.getDistance(DistanceUnit.METER)));
        telemetry.addData("RightRange", String.format("%.01f in", RightDistance.getDistance(DistanceUnit.INCH)));


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

        telemetry.update();
        waitForStart();
        runtime.reset();
        ClawHand.setPosition(0.1);

        short pixelLocal = -1;

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            telemetry.update();
            resetEncoders();

            // go forward a little not looking at the distance value
            while (opModeIsActive() && MotorFL.getCurrentPosition() < 1600) {            // 1830
                MotorFL.setVelocity(1000);
                MotorFR.setVelocity(-1000);
                MotorBL.setVelocity(-1000);
                MotorBR.setVelocity(-1000);
            }

            zeroMotors();
            resetEncoders();
//            telemetry.addData("Done with that Waiting ", "for 2 seconds"); // temp
//            telemetry.update();
//            sleep(2000);

            boolean FoundLeft = false;
            boolean FoundRight = false;
            // go forward more while looking at the distance value
            while (opModeIsActive() && MotorFL.getCurrentPosition() < 600) { // go forward some distance
                if (LeftDistance.getDistance(DistanceUnit.MM) < 250) {
                    FoundLeft = true;
                    break;
                }
                if (RightDistance.getDistance(DistanceUnit.MM) < 250) {
                    FoundRight = true;
                    break;
                }
                telemetry.addData("FoundLeft", FoundLeft);
                telemetry.addData("FoundRight", FoundRight);
                telemetry.addData("LeftRange", String.format("%.01f mm", LeftDistance.getDistance(DistanceUnit.MM)));
                telemetry.addData("RightRange", String.format("%.01f mm", RightDistance.getDistance(DistanceUnit.MM)));
                telemetry.update();
                MotorFL.setVelocity(1000);
                MotorFR.setVelocity(-1000);
                MotorBL.setVelocity(-1000);
                MotorBR.setVelocity(-1000);
            }
            zeroMotors();
            sleep(3000);
            if (FoundLeft) {
                telemetry.addData("FoundLeft", FoundLeft);
                telemetry.update();
            }
            if (FoundRight) {
                telemetry.addData("FoundRight", FoundRight);
                telemetry.update();
            }
//            telemetry.addData("rotations", toString().valueOf(MotorFL.getCurrentPosition()));
//            telemetry.update();
//            sleep(1000000);
            if (FoundLeft) {
                pixelLocal = 2; // left!!
                runtime.reset();
                resetEncoders();
                while (opModeIsActive() && MotorFL.getCurrentPosition() > -1400) {
                    MotorFL.setVelocity(-1000);
                    MotorBR.setVelocity(-1000);
                }
                zeroMotors();
                resetEncoders();
                while (opModeIsActive() && MotorFL.getCurrentPosition() < 160) {
                    MotorFL.setVelocity(1000);
                    MotorFR.setVelocity(-1000);
                    MotorBL.setVelocity(-1000);
                    MotorBR.setVelocity(-1000);
                }
                zeroMotors();
                resetEncoders();
                zeroMotors();
                resetEncoders();
                ClawHand.setPosition(0.4);
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < 2) {
                    ClawHand.setPosition(0.4);
                }
            }

            if (FoundRight) {
                pixelLocal = 3; // right?
                telemetry.update();
                resetEncoders();
                while (opModeIsActive() && MotorFL.getCurrentPosition() < 900) {
                    MotorBR.setVelocity(1000);
                    MotorFL.setVelocity(1000);
                }
                zeroMotors();
                resetEncoders();
                while (opModeIsActive() && MotorFL.getCurrentPosition() < 100) {
                    MotorFL.setVelocity(1000);
                    MotorFR.setVelocity(-1000);
                    MotorBL.setVelocity(-1000);
                    MotorBR.setVelocity(-1000);
                }
                zeroMotors();
                ClawHand.setPosition(0.4);
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < 2) {
                    ClawHand.setPosition(0.4);
                }
            }
            // not found assume center?
            if (!FoundLeft && !FoundRight) {
                zeroMotors();
                pixelLocal = 1; // Left
                resetEncoders();
                while (opModeIsActive() && MotorFL.getCurrentPosition() < 280) {
                    MotorFL.setVelocity(1000);
                    MotorFR.setVelocity(-1000);
                    MotorBL.setVelocity(-1000);
                    MotorBR.setVelocity(-1000);
                }
                zeroMotors();
                ClawHand.setPosition(0.4);
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() > 2) {
                    ClawHand.setPosition(0.4);
                }
            }


            // raise arm after dropping purple
            ArmR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            ArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (ArmR.getCurrentPosition() < 180) {
                ArmL.setVelocity(-1000);
                ArmR.setVelocity(1000);
            }
            ArmL.setVelocity(0);
            ArmR.setVelocity(0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 2) {
                ClawHand.setPosition(0.1);
            }
            // raise up more after picking up yellow pixel
            ArmR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            ArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (ArmR.getCurrentPosition() < 300) {
                ArmL.setVelocity(-1000);
                ArmR.setVelocity(1000);
            }
            ArmL.setVelocity(0);
            ArmR.setVelocity(0);

            // done finding location and dropping purple
            // now reverse from whatever location we are currently at

            //reverse
            zeroMotors();
            resetEncoders();
            if (FoundLeft) {
                while (opModeIsActive() && MotorFL.getCurrentPosition() > -160) {
                    MotorFL.setVelocity(-1000);
                    MotorFR.setVelocity(1000);
                    MotorBL.setVelocity(1000);
                    MotorBR.setVelocity(1000);
                }
                zeroMotors();
                resetEncoders();
                // move left
            }


            // turn based on location
            // -90 if center
            // 180 if right
            // 0 if left

            // now facing left
            // move left to wall till distance < ?
            // more

        }
        visionPortal.close();
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

    private void initAprilTag() {

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}   // end class
