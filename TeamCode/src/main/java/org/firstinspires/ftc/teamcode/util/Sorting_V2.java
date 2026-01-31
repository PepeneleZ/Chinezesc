package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.MultiplePIDF;

import org.firstinspires.ftc.teamcode.util.Constants.TRANSFER_POS;
import org.firstinspires.ftc.teamcode.util.Constants.COLORS;
import org.firstinspires.ftc.teamcode.util.Constants.MOVING_STATES;


public class Sorting_V2 implements Updateable {

    /// References
    CRServo blade1, blade2;
    Servo transfer1, transfer2;
    DcMotorEx encoder;
    VoltageSensor voltage;
    MultiplePIDF pid = new MultiplePIDF(0.0001d,0.000002d,0.000000d,0);
    Telemetry telemetry;

    /// States
    TRANSFER_POS transfer_pos = TRANSFER_POS.DOWN;
    COLORS[] storage = new COLORS[4];
    COLORS[] launching_order = new COLORS[4];
    MOVING_STATES next_moving_state = MOVING_STATES.NOTHING;
    MOVING_STATES current_moving_state = MOVING_STATES.NOTHING;
    MOVING_STATES last_moving_state = MOVING_STATES.NOTHING;


    public Sorting_V2(HardwareMap hwmap, Telemetry telemetry, VoltageSensor voltageSensor){
        blade1 = hwmap.get(CRServo.class, HardwareConfig.sorting1);
        blade2 = hwmap.get(CRServo.class, HardwareConfig.sorting2);

        transfer1 = hwmap.get(Servo.class, HardwareConfig.transfer_servo1);
        transfer2 = hwmap.get(Servo.class, HardwareConfig.transfer_servo2);

        encoder = hwmap.get(DcMotorEx.class, HardwareConfig.right_lifter);

        this.voltage = voltageSensor;
        this.telemetry = telemetry;

        transfer1.setDirection(Servo.Direction.REVERSE);

        transfer1.setPosition(0);
        transfer2.setPosition(0);

        blade1.setDirection(DcMotorSimple.Direction.FORWARD);
        blade2.setDirection(DcMotorSimple.Direction.FORWARD);

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pid.addPidCoefficients(0.0002,0.000002d,0.000000d); // for third of a rotation
        pid.addPidCoefficients(0.000163d,0.0000062d,0.00000091d); // for sixth of a rotation


    }





    @Override
    public void update() {

    }
}
