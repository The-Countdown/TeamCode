package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DriveMode2")
public class Drive extends LinearOpMode {
  private TouchSensor Touch;
  private DcMotor MotorFL;
  private DcMotor MotorFR;
  private DcMotor MotorBL;
  private DcMotor MotorBR;
  private Servo ServoArm;
  private Servo ServoClaw;
  private DcMotor Arm1;
  private DcMotor Arm2;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
  Touch = hardwareMap.get(TouchSensor.class, "Touch");
  MotorFL = hardwareMap.get(DcMotor.class, "MotorFL");
  MotorFR = hardwareMap.get(DcMotor.class, "MotorFR");
  MotorBL = hardwareMap.get(DcMotor.class, "MotorBL");
  MotorBR = hardwareMap.get(DcMotor.class, "MotorBR");
  Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
  Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
  ServoArm = hardwareMap.get(Servo.class, "ServoArm");
  ServoClaw = hardwareMap.get(Servo.class, "ServoClaw");
 
  
  
  telemetry.addData("Status", "Initialized");
        
  MotorFL.setDirection(DcMotorSimple.Direction.FORWARD);
  MotorFR.setDirection(DcMotorSimple.Direction.FORWARD);
  MotorBL.setDirection(DcMotorSimple.Direction.FORWARD);
  MotorBR.setDirection(DcMotorSimple.Direction.FORWARD);
  Arm1.setDirection(DcMotorSimple.Direction.FORWARD);
  Arm2.setDirection(DcMotorSimple.Direction.FORWARD);
  
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
    
  telemetry.update();
    waitForStart();
    while (opModeIsActive()) {
      
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
      boolean ButtonTouch = Touch.isPressed();
      float TriggerR22 = gamepad2.right_trigger;
      float TriggerL22 = gamepad2.left_trigger;
      
      MotorPowerRX1 = MotorPowerRX1 * BSpeed;
      MotorPowerLX1 = MotorPowerLX1 * BSpeed;
      MotorPowerLY1 = MotorPowerLY1 * BSpeed;
      
      telemetry.addData("Status lx1", MotorPowerLX1);
      telemetry.addData("Status ly1", MotorPowerLY1);
      telemetry.addData("Status rx1", MotorPowerRX1);
      telemetry.addData("Status ButtonX1", ButtonX1);
      telemetry.addData("Status rightTrigger1", TriggerR21);
      telemetry.addData("Status Mode1", ButtonOptions2);
      telemetry.addData("Status Share1", ButtonShare2);
      telemetry.addData("Status ModeToggle1", ModeToggle);
      telemetry.addData("Status Touch", ButtonTouch);
      telemetry.addData("Status lol", Arm1.getCurrentPosition());
      telemetry.update();
      
      if (MotorPowerRX1 > 1) {
        MotorPowerRX1 = 1;
      }
      if (MotorPowerLX1 > 1) {
        MotorPowerLX1 = 1;
      }
      if (MotorPowerLY1 > 1) {
        MotorPowerLY1 = 1;
      }
      
      if (ModeToggle) {
        MotorPowerRX1 = MotorPowerRX1 / 1.5;
        MotorPowerLX1 = MotorPowerLX1 / 1.5;
        MotorPowerLY1 = MotorPowerLY1 / 1.5;
        TriggerR21 = TriggerR21 / 2;
        TriggerL21 = TriggerL21 / 2;
      }
        
      if (ButtonB1) {
        ModeToggle = true;
      }
      
      if (ButtonY1) {
        ModeToggle = false;
      }
      
      if (ButtonRBump2) {
        Arm2.setPower(-1);
      }
      
      if (ButtonLBump2) {
        Arm2.setPower(1);
      }
      
      if (ButtonDPup2) {
        Arm1.setPower(1);
      }
      
      if (ButtonDPdown2) {
        Arm1.setPower(-1);
        
      }
      
      
      if (ButtonDPleft2) {
        ServoArm.setPosition(0);
      }
      
      if (ButtonDPright2) {
        ServoArm.setPosition(1);
        
      }
      
      if (ButtonA2) {
        ServoClaw.setPosition(1);
      }
      
      if (ButtonX2) {
        ServoClaw.setPosition(0);
      }
      
      if (MotorPowerLY1 > -0) { // move forward
        MotorFL.setPower(MotorPowerLY1);
        MotorFR.setPower(-MotorPowerLY1);
        MotorBL.setPower(MotorPowerLY1);
        MotorBR.setPower(-MotorPowerLY1);
      }
      
      if (MotorPowerLY1 < 0) { // move backword
        MotorFL.setPower(MotorPowerLY1);
        MotorFR.setPower(-MotorPowerLY1);
        MotorBL.setPower(MotorPowerLY1);
        MotorBR.setPower(-MotorPowerLY1);
      }
      
      // MotorPowerLX
      if (MotorPowerRX1 > 0) { // move left
        MotorFL.setPower(-MotorPowerRX1);
        MotorFR.setPower(-MotorPowerRX1);
        MotorBL.setPower(MotorPowerRX1);
        MotorBR.setPower(MotorPowerRX1);
      }
      
      // MotorPowerLX
      if (MotorPowerRX1 < -0) { // move right
        MotorFL.setPower(-MotorPowerRX1);
        MotorFR.setPower(-MotorPowerRX1);
        MotorBL.setPower(MotorPowerRX1);
        MotorBR.setPower(MotorPowerRX1);
      }
      
      // MotorPowerRX
      if (TriggerL21 > 0) { // rotate left
        MotorFL.setPower(TriggerL21);
        MotorFR.setPower(TriggerL21);
        MotorBL.setPower(TriggerL21);
        MotorBR.setPower(TriggerL21);
      }
      
      // MotorPowerLX
      if (TriggerR21 > 0) { // rotate right
        MotorFL.setPower(-TriggerR21);
        MotorFR.setPower(-TriggerR21);
        MotorBL.setPower(-TriggerR21);
        MotorBR.setPower(-TriggerR21);
      }
      
      MotorFL.setPower(0);
      MotorFR.setPower(0);
      MotorBL.setPower(0);
      MotorBR.setPower(0);
      Arm1.setPower(0);
      Arm2.setPower(0);

      
      

    
      
    }
  }
}



