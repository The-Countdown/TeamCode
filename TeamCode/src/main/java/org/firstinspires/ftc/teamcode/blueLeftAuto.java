package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Blue Left Auto")
public class blueLeftAuto extends LinearOpMode {

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
    private BNO055IMU imu;
    private ColorSensor col;
    ElapsedTime runtime = new ElapsedTime();
    Orientation robotOrientation;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
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
        ClawHand = hardwareMap.get(Servo.class, "ClawHand");
        servoTest = hardwareMap.get(Servo.class, "ServoTest");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        col = hardwareMap.get(ColorSensor.class, "col");

        imu.initialize(parameters);


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

        telemetry.update();
        waitForStart();
        runtime.reset();
        ClawHand.setPosition(0.1);

        short pixelLocal = -1;

        //initTfod();
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
//        while (opModeIsActive()) {
//            if (gamepad1.a) {
//                visionPortal.close();
//                sleep(50);
//                initAprilTag();
//                visionPortal.resumeStreaming();
//            }
//            if (gamepad1.b) {
//                visionPortal.close();
//                sleep(50);
//                initTfod();
//                visionPortal.resumeStreaming();
//            }
//            telemetry.addLine(toString().valueOf(robotOrientation.firstAngle));
//            telemetry.addLine(toString().valueOf(robotOrientation.secondAngle));
//            telemetry.addLine(toString().valueOf(robotOrientation.thirdAngle));
//            telemetry.update();
//        }

        if (opModeIsActive()) {
            telemetry.update();
            resetEncoders();
            while (opModeIsActive() && MotorFL.getCurrentPosition() < 1830) {
                MotorFL.setVelocity(1000);
                MotorFR.setVelocity(-1000);
                MotorBL.setVelocity(-1000);
                MotorBR.setVelocity(-1000);
            }
            zeroMotors();
            telemetry.addData("rotations", toString().valueOf(MotorFL.getCurrentPosition()));
            telemetry.update();
            resetEncoders();
            while (opModeIsActive() && MotorFL.getCurrentPosition() < 300) {
                MotorFL.setVelocity(1000);
                MotorBR.setVelocity(1000);
            }
            zeroMotors();
            telemetry.update();
            visionPortal.resumeStreaming();
            runtime.reset();
            boolean found = false;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            while (opModeIsActive() && runtime.seconds() < 1) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == 21) {
                        found = true;
                    } else {
                        found = false;
                    }
                    if (found) {
                        break;
                    }
                }
                if (found) {
                    break;
                }
                currentDetections = aprilTag.getDetections();
            }
            if (found) {
                pixelLocal = 2;
                telemetry.update();
                resetEncoders();
                while (opModeIsActive() && MotorFL.getCurrentPosition() > -300) {
                    MotorFL.setVelocity(-1000);
                    MotorBR.setVelocity(-1000);
                }
                zeroMotors();
                resetEncoders();
                while (opModeIsActive() && MotorFL.getCurrentPosition() < 200) {
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

            if (pixelLocal == -1) {
                resetEncoders();
                while (opModeIsActive() && MotorFL.getCurrentPosition() > -800) {
                    MotorFL.setVelocity(-1000);
                    MotorBR.setVelocity(-1000);
                }
                zeroMotors();
                runtime.reset();
                found = false;
                currentDetections = aprilTag.getDetections();
                while (opModeIsActive() && runtime.seconds() < 1) {
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.id == 21) {
                            found = true;
                        } else {
                            found = false;
                        }
                        if (found) {
                            break;
                        }
                    }
                    if (found) {
                        break;
                    }
                    currentDetections = aprilTag.getDetections();
                }
                if (found) {
                    pixelLocal = 1;
                    telemetry.update();
                    robotOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    while (opModeIsActive() && robotOrientation.firstAngle < 60) {
                        MotorFL.setVelocity(-1000);
                        MotorBR.setVelocity(-1000);
                        robotOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        telemetry.addLine(toString().valueOf(robotOrientation.firstAngle));
                        telemetry.update();
                    }
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
            }
            if (pixelLocal == -1) {
                runtime.reset();
                resetEncoders();
                while (opModeIsActive() && MotorFL.getCurrentPosition() < 1950) {
                    MotorFL.setVelocity(1000);
                    MotorBR.setVelocity(1000);
                }
                zeroMotors();
                pixelLocal = 3;
                resetEncoders();
                while (opModeIsActive() && MotorFL.getCurrentPosition() < 300) {
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

            ArmR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            ArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (ArmR.getCurrentPosition() < 200) {
                ArmL.setVelocity(-1000);
                ArmR.setVelocity(1000);
            }
            ArmL.setVelocity(0);
            ArmR.setVelocity(0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 2) {
                ClawHand.setPosition(0.1);
            }
            ArmR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            ArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (ArmR.getCurrentPosition() < 300) {
                ArmL.setVelocity(-1000);
                ArmR.setVelocity(1000);
            }
            ArmL.setVelocity(0);
            ArmR.setVelocity(0);
            int count = -100;
            if (pixelLocal == 1) {
                count = -100;
            }
            if (pixelLocal == 3) {
                count = -300;
            }
            resetEncoders();
            while (opModeIsActive() && MotorFL.getCurrentPosition() > count) {
                MotorFL.setVelocity(-1000);
                MotorFR.setVelocity(1000);
                MotorBL.setVelocity(1000);
                MotorBR.setVelocity(1000);
            }
            zeroMotors();
//            count = 1400;
//            if (pixelLocal == 1) {
//                count = 2250;
//            }
//            if (pixelLocal == 3) {
//                count = 250;
//            }
//            resetEncoders();
//            while (opModeIsActive() && MotorFL.getCurrentPosition() < count) {
//                MotorFL.setVelocity(1000);
//                MotorBR.setVelocity(1000);
//                telemetry.update();
//            }

            robotOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            while (opModeIsActive() && robotOrientation.firstAngle < 80) {
                MotorFL.setVelocity(-1000);
                MotorBR.setVelocity(-1000);
                robotOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addLine(toString().valueOf(robotOrientation.firstAngle));
                telemetry.update();
            }

            zeroMotors();
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1) {
                MotorFL.setVelocity(1000);
                MotorFR.setVelocity(-1000);
                MotorBL.setVelocity(-1000);
                MotorBR.setVelocity(-1000);
                telemetry.update();
            }
            resetEncoders();
            while (opModeIsActive() && col.blue() < 800 && MotorFL.getCurrentPosition() < 2100 || MotorFL.getCurrentPosition() < 1400) {
                MotorFL.setVelocity(1000);
                MotorFR.setVelocity(-1000);
                MotorBL.setVelocity(-1000);
                MotorBR.setVelocity(-1000);
                telemetry.addData("color", toString().valueOf(col.blue()));
                telemetry.update();
            }
            resetEncoders();
            while (opModeIsActive() && MotorFL.getCurrentPosition() > -400) {
                MotorFL.setVelocity(-1000);
                MotorFR.setVelocity(1000);
                MotorBL.setVelocity(1000);
                MotorBR.setVelocity(1000);
                telemetry.update();
            }
            zeroMotors();
            robotOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            while (opModeIsActive() && robotOrientation.firstAngle < 87) {
                MotorFL.setVelocity(-300);
                MotorBR.setVelocity(-300);
                robotOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addLine(toString().valueOf(robotOrientation.firstAngle));
                telemetry.update();
            }
            robotOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (robotOrientation.firstAngle > 100) {
                while (opModeIsActive() && robotOrientation.firstAngle > 93) {
                    MotorFL.setVelocity(300);
                    MotorBR.setVelocity(300);
                    robotOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addLine(toString().valueOf(robotOrientation.firstAngle));
                    telemetry.update();
                }
            }
            currentDetections = aprilTag.getDetections();
            runtime.reset();
            while (opModeIsActive() && currentDetections.size() != 0 || runtime.seconds() < 0.5) {
                MotorFL.setVelocity(-600);
                MotorFR.setVelocity(-600);
                MotorBL.setVelocity(-600);
                MotorBR.setVelocity(600);
                currentDetections = aprilTag.getDetections();
                if (currentDetections.size() != 0) {
                    runtime.reset();
                }
            }
            zeroMotors();
            found = false;
            if (pixelLocal != 1) {
                MotorFL.setVelocity(300);
                MotorFR.setVelocity(300);
                MotorBL.setVelocity(300);
                MotorBR.setVelocity(-300);
                while (true) {
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.id == pixelLocal) {
                            found = true;
                        } else {
                            found = false;
                        }
                        if (found) {
                            break;
                        }
                    }
                    if (found) {
                        zeroMotors();
                        break;
                    }
                    currentDetections = aprilTag.getDetections();
                }
            }
            ClawArm.setPosition(0.15);
            resetEncoders();
            while (opModeIsActive() && MotorFL.getCurrentPosition() < 50) {
                MotorFL.setVelocity(300);
                MotorFR.setVelocity(300);
                MotorBL.setVelocity(300);
                MotorBR.setVelocity(-300);
            }
            zeroMotors();
            resetEncoders();
            while (opModeIsActive() && col.blue() < 800 && MotorFL.getCurrentPosition() < 200) {
                MotorFL.setVelocity(1000);
                MotorFR.setVelocity(-1000);
                MotorBL.setVelocity(-1000);
                MotorBR.setVelocity(-1000);
                telemetry.addData("color", toString().valueOf(col.blue()));
                telemetry.update();
            }
            resetEncoders();
            while (opModeIsActive() && MotorFL.getCurrentPosition() < 400) {
                MotorFL.setVelocity(1000);
                MotorFR.setVelocity(-1000);
                MotorBL.setVelocity(-1000);
                MotorBR.setVelocity(-1000);
                telemetry.update();
            }
            zeroMotors();
            ClawHand.setPosition(0.4);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1) {
                telemetry.update();
            }
            resetEncoders();
            while (opModeIsActive() && MotorFL.getCurrentPosition() > -50) {
                MotorFL.setVelocity(-1000);
                MotorFR.setVelocity(1000);
                MotorBL.setVelocity(1000);
                MotorBR.setVelocity(1000);
                telemetry.update();
            }
            zeroMotors();
        }
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
