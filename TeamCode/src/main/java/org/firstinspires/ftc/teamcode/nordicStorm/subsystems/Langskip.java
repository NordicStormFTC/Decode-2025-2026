package org.firstinspires.ftc.teamcode.nordicStorm.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * The goal here is to have only one robot info class instantiated
 * in our OpModes. Langskip means 'boat' in old norse, and publicly
 * contains all of our subsystems. Users should not instantiate
 * individual subsystems in OpModes and access them through langskip.
 */
public class Langskip {

    public final Intake intake;


    public final DriveTrain driveTrain;

    public final InnerSubsystem innerSubsystem;

    /**
     *
     * @param hardwareMap the hardware map for our subsystems to use. This provides the same instance of the hardware map to all subsystems
     */
    public Langskip(@NonNull final HardwareMap hardwareMap) {

        intake = new Intake(hardwareMap);

        driveTrain = new DriveTrain(hardwareMap);

        innerSubsystem = new InnerSubsystem(hardwareMap);
    }
}