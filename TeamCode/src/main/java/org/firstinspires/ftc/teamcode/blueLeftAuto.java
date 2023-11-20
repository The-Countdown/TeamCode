package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.io.File;
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
    private Orientation robotOrientation;
    ElapsedTime runtime = new ElapsedTime();

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
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

        BNO055IMU.Parameters imuParameters;

        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);

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

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
        File calibrationFile = AppUtil.getInstance().getSettingsFile("IMUCalibration.json");
        ReadWriteFile.writeFile(calibrationFile, calibrationData.serialize());

        robotOrientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        if (opModeIsActive()) {
            telemetryTfod();
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
            while (opModeIsActive() && MotorFL.getCurrentPosition() < 350) {
                MotorFL.setVelocity(1000);
                MotorBR.setVelocity(1000);
            }
            zeroMotors();
            telemetry.update();
            visionPortal.resumeStreaming();
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 2) {
                telemetryTfod();
            }
            telemetry.addData("pixel local", toString().valueOf(pixelLocal));
            telemetry.update();
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            if (currentRecognitions.size() >= 1) {
                pixelLocal = 2;
                telemetry.update();
                resetEncoders();
                while (opModeIsActive() && MotorFL.getCurrentPosition() > -350) {
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
                while (opModeIsActive() && MotorFL.getCurrentPosition() < 700) {
                    MotorFL.setVelocity(1000);
                    MotorBR.setVelocity(1000);
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
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < 2) {
                    telemetryTfod();
                }
                currentRecognitions = tfod.getRecognitions();
                if (currentRecognitions.size() >= 1) {
                    pixelLocal = 1;
                    telemetry.update();
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
                while (opModeIsActive() && MotorFL.getCurrentPosition() > -1900) {
                    MotorFL.setVelocity(-1000);
                    MotorBR.setVelocity(-1000);
                }
                zeroMotors();
                pixelLocal = 3;
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
                while (opModeIsActive() && runtime.seconds() > 2) {
                    ClawHand.setPosition(0.4);
                }
            }
            ArmR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            ArmR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (ArmR.getCurrentPosition() < 120) {
                ArmL.setVelocity(-1000);
                ArmR.setVelocity(1000);
            }
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() > 2) {
                ClawHand.setPosition(0.1);
            }
            robotOrientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            while (opModeIsActive() && robotOrientation.firstAngle < 89) {
                robotOrientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                MotorFL.setVelocity(1000);
                MotorBR.setVelocity(1000);
            }
            zeroMotors();

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

}   // end class
