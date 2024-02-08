package org.firstinspires.ftc.teamcode.subsystems;
public class Drive extends Robot.Components {
    public void stop() {
        MotorFL.setVelocity(0);
        MotorFR.setVelocity(0);
        MotorBL.setVelocity(0);
        MotorBR.setVelocity(0);
    }

    public void move(double forwardsVelocity, double sidewaysVelocity) {
        MotorFL.setVelocity(forwardsVelocity + sidewaysVelocity);
        MotorFR.setVelocity(forwardsVelocity + sidewaysVelocity);
        MotorBL.setVelocity(forwardsVelocity + sidewaysVelocity);
        MotorBR.setVelocity(forwardsVelocity + sidewaysVelocity);
    }
    public void move(double forwardsVelocity, double sidewaysVelocity, double amount) {
        move(forwardsVelocity, sidewaysVelocity);

        while (MotorFL.getCurrentPosition() < amount) {
            //wait
        }

        stop();
    }

    public void move(double forwardsVelocity, double sidewaysVelocity, double turnR, double turnL){
        MotorFL.setVelocity(forwardsVelocity + sidewaysVelocity - turnR + turnL);
        MotorFR.setVelocity(forwardsVelocity + sidewaysVelocity + turnR - turnL);
        MotorBL.setVelocity(forwardsVelocity + sidewaysVelocity - turnR + turnL);
        MotorBR.setVelocity(forwardsVelocity + sidewaysVelocity + turnR - turnL);
    }

    public void moveField(double forwardsVelocity, double sidewaysVelocity){
        // get IMU and clamp to 0-360
        double angle = imu.getAngularOrientation().firstAngle;
        if (angle < 0) {
            angle += 360;
        }

        //adjust motor powers
        double angleInRadians  = Math.toRadians(angle);
        double newForward = forwardsVelocity * Math.cos(angleInRadians) +
                sidewaysVelocity * Math.sin(angleInRadians);
        sidewaysVelocity = -forwardsVelocity * Math.sin(angleInRadians) +
                sidewaysVelocity * Math.cos(angleInRadians);
        forwardsVelocity = newForward;

        MotorFL.setVelocity(forwardsVelocity + sidewaysVelocity);
        MotorFR.setVelocity(forwardsVelocity + sidewaysVelocity);
        MotorBL.setVelocity(forwardsVelocity + sidewaysVelocity);
        MotorBR.setVelocity(forwardsVelocity + sidewaysVelocity);
    }

    public void moveField(double forwardsVelocity, double sidewaysVelocity, double amount) {
        moveField(forwardsVelocity, sidewaysVelocity);

        while (MotorFL.getCurrentPosition() < amount) {
            //wait
        }

        stop();
    }

    public void moveField(double forwardsVelocity, double sidewaysVelocity, double turnR, double turnL){
        // get IMU and clamp to 0-360
        double angle = imu.getAngularOrientation().firstAngle;
        if (angle < 0) {
            angle += 360;
        }

        //adjust motor powers
        double angleInRadians  = Math.toRadians(angle);
        double newForward = forwardsVelocity * Math.cos(angleInRadians) +
                sidewaysVelocity * Math.sin(angleInRadians);
        sidewaysVelocity = -forwardsVelocity * Math.sin(angleInRadians) +
                sidewaysVelocity * Math.cos(angleInRadians);
        forwardsVelocity = newForward;

        MotorFL.setVelocity(forwardsVelocity + sidewaysVelocity - turnR + turnL);
        MotorFR.setVelocity(forwardsVelocity + sidewaysVelocity + turnR - turnL);
        MotorBL.setVelocity(forwardsVelocity + sidewaysVelocity - turnR + turnL);
        MotorBR.setVelocity(forwardsVelocity + sidewaysVelocity + turnR - turnL);
    }

    public void turnToAngle(double velocity, double targetAngle) {
        double angle = imu.getAngularOrientation().firstAngle;

        while (angle < 0) {
            angle += 360;
        }

        while (angle > 360) {
            angle -= 360;
        }

        double error = angle - targetAngle;
        while (error < -180) {
            error += 360;
        }

        while (error > 180) {
            error -= 360;
        }

        while (error > 1) {
            angle = imu.getAngularOrientation().firstAngle;
            while (angle < 0) {
                angle += 360;
            }

            while (angle > 360) {
                angle -= 360;
            }

            error = angle - targetAngle;
            while (error < -180) {
                error += 360;
            }

            while (error > 180) {
                error -= 360;
            }

            if (error > 0) {
                MotorFL.setVelocity(-velocity);
                MotorFR.setVelocity(velocity);
                MotorBL.setVelocity(-velocity);
                MotorBR.setVelocity(velocity);
            } else {
                MotorFL.setVelocity(velocity);
                MotorFR.setVelocity(-velocity);
                MotorBL.setVelocity(velocity);
                MotorBR.setVelocity(-velocity);
            }
        }
        stop();
    }
}
