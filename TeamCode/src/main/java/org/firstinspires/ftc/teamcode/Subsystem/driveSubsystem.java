package org.firstinspires.ftc.teamcode.subsystems;

@Configurable
public class DriveSubsystem {
    private DcMotorEx fl, fr, bl, br;
}

public void initialize(HardwareMap hwMap) {
    // Motor Setup
    fl = hwMap.get(DcMotorEx.class, "left_front_motor");
    fr = hwMap.get(DcMotorEx.class, "right_front_motor");
    bl = hwMap.get(DcMotorEx.class, "left_back_motor");
    br = hwMap.get(DcMotorEx.class, "right_back_motor");

    fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    br.setDirection(DcMotorEx.Direction.REVERSE);
    fl.setDirection(DcMotorEx.Direction.REVERSE);
    bl.setDirection(DcMotorEx.Direction.REVERSE);
    fr.setDirection(DcMotorEx.Direction.REVERSE);