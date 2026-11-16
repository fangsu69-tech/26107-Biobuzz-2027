package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {
    private DcMotor intakeMotor;
    private CRServo transferServo1, transferServo2;
    public static double intakeSpeed = 0.8;


    public void initialize(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        transferServo1 = hwMap.get(CRServo.class, "left_intake_servo");
        transferServo2 = hwMap.get(CRServo.class, "right_intake_servo");

    }
    public void runIntake(){
        intakeMotor.setPower(intakeSpeed);
    }

    public void retractIntake() {
        intakeMotor.setPower(-intakeSpeed);
    }

    public void stopIntake(){
        intakeMotor.setPower(0);
    }
    public void runTransfer(){
        transferServo1.setPower(-1);
        transferServo2.setPower(1);

    }

    public void retractTransfer(){
        transferServo1.setPower(1);
        transferServo2.setPower(-1);
    }

    public void stopTransfer() {
        transferServo1.setPower(0);
        transferServo2.setPower(0);
    }

}

