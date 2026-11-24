package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;
import java.util.List;

public class Robot {
    // Subsystems
    public DriveSubsystem drive = new DriveSubsystem();
    public IntakeSubsystem intake = new IntakeSubsystem();
    public ShooterSubsystem shooter = new ShooterSubsystem();

    // Hardware Hubs for Bulk Reads
    private List<LynxModule> allHubs;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        //Initialize Telemetry Utility
        TelemetryUtils.init(telemetry);

        // Initialize Drive Subsystem
        drive.initialize(hwMap);
        // Initialize Intake Subsystem
        intake.initialize(hwMap);
        // Initialize Shooter Subsystem
        shooter.initialize(hwMap);
        // Set up Bulk Reads for all REV Hubs
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    /**
     * Call this at the start of every loop() to refresh sensor data.
     */
    public void clearCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}