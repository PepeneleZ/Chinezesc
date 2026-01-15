package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.tests.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.COLORS;


public class StorageSlot {
    public ColorSensor colorSensor;
    public Servo transferServo;
    public COLORS color = COLORS.EMPTY;
    public boolean isUP = false;
    public boolean sensorEnabled = false;

    public StorageSlot(HardwareMap hardwareMap, int index){
        colorSensor = hardwareMap.get(ColorSensor.class, HardwareConfig.color_sensor+index);
        transferServo = hardwareMap.get(Servo.class, HardwareConfig.transfer+index);
        transferServo.setPosition(Constants_Enums.TRANSFER_POS.DOWN.val);
        sensorEnabled = true;
    }

    public void checkColor(){
        if (colorSensor == null) {
            color = COLORS.EMPTY;
            return;
        }
        else if (!sensorEnabled)
            return;
        color = ColorFunctions.getColor(colorSensor.red(),colorSensor.green(),colorSensor.blue());
    }
    public void toggle_transfer(){
        if (isUP)
            transferServo.setPosition(Constants_Enums.TRANSFER_POS.UP.val);
        else
            transferServo.setPosition(Constants_Enums.TRANSFER_POS.DOWN.val);
        isUP = !isUP;
    }
    public void toggle_transfer(boolean isUP){
        this.isUP = isUP;
        if (isUP)
            transferServo.setPosition(Constants_Enums.TRANSFER_POS.UP.val);
        else
            transferServo.setPosition(Constants_Enums.TRANSFER_POS.DOWN.val);
    }

    public void toggle_sensor(){
        colorSensor.enableLed(!sensorEnabled);
        sensorEnabled = !sensorEnabled;
    }
    public void toggle_sensor(boolean sensorEnabled){
        colorSensor.enableLed(sensorEnabled);
        this.sensorEnabled = sensorEnabled;
    }

    public void update(){
        checkColor();
    }
}
