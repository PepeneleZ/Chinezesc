package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants.COLORS;
import org.firstinspires.ftc.teamcode.util.Constants.MOTIF;
import org.firstinspires.ftc.teamcode.util.Constants.MOVING_STATES;


@Deprecated
public class Storage implements Updateable {
    public StorageSlot[] storageSlots = new StorageSlot[4];
    public MOTIF motif = MOTIF.PPG;
    public MOVING_STATES state = MOVING_STATES.NOTHING;
    public Intake intake;
    private ElapsedTime transferTimer;
    private Telemetry telemetry;
    private int[] shooting_order = new int[4];
    public int shooting_index = 1;
    public int shift=0;

    public Storage(HardwareMap hardwareMap, Telemetry telemetry, Intake intake){
        for (int i=1;i<=3;i++){
            storageSlots[i] = new StorageSlot(hardwareMap,i);
        }
        this.intake = intake;
        this.telemetry = telemetry;
    }

    public void setState(MOVING_STATES state) {
        this.state = state;
        for(StorageSlot slot : storageSlots){
            if (state == MOVING_STATES.WAITING_INTAKE){
                slot.toggle_sensor(true);
            }
            else if (state == MOVING_STATES.NOTHING) {
                slot.toggle_sensor(false);
            }
            else if (state == MOVING_STATES.SHOOTING) {
                slot.toggle_sensor(false);
                startShooting();
            }

        }
    }

    @Override
    public void update() {
        switch (state){
            case WAITING_INTAKE:
                if(isFull()){
                    state = MOVING_STATES.NOTHING;
                    intake.toggle(Constants.INTAKE_STATES.STOPPED);
                }
                break;

            case SHOOTING:
                updateShooting();
                break;
        }

        for(StorageSlot slot : storageSlots){
            slot.update();
        }
        telemetryData();
    }
    public void startShooting(){
        shooting_order = getShootingOrder();
        shooting_index = 0;
        transferTimer.reset();
        state = MOVING_STATES.SHOOTING;
    }

    private void updateShooting(){
        if(shooting_index >= shooting_order.length){
            state = MOVING_STATES.NOTHING;
            return;
        }

        int slot = shooting_order[shooting_index];

        if(transferTimer.seconds() < 0.2 && shooting_index != 0) return;

        if(!storageSlots[slot].isUP){
            toggle_transfer(slot, true);
        } else {
            toggle_transfer(slot, false);
            shooting_index++;
        }
    }

    public boolean isFull(){
        boolean isFull=true;
        for(StorageSlot slot : storageSlots){
            if (slot.color == COLORS.EMPTY) {
                isFull = false;
                break;
            }
        }
        return isFull;
    }

    public void toggle_transfer(int index){ //toggle
        storageSlots[index].toggle_transfer(!storageSlots[index].isUP); // sa schimbam numele in storage slots la toggle_servo sau cv
        transferTimer.reset();
        transferTimer.startTime();
        if(!storageSlots[index].isUP)
            return;
        if(Turret.turret_launcher_state != Constants.TURRET_LAUNCH_SPEEDS.STOPPED || Turret.feedforwardController.targetVelocity>500) {
            storageSlots[index].color = COLORS.EMPTY;
        }

    }
    public void toggle_transfer(int index, boolean toggle){ //toggle
        storageSlots[index].toggle_transfer(toggle);
        transferTimer.reset();
        transferTimer.startTime();
        if(!toggle)
            return;
        if(Turret.turret_launcher_state != Constants.TURRET_LAUNCH_SPEEDS.STOPPED || Turret.feedforwardController.targetVelocity>500) {
            storageSlots[index].color = COLORS.EMPTY;
        }

    }
    public int[] getShootingOrder(){
        int green_pos=0, current_motif = motif.val, cont_purple = 0, current_shift=shift%4; // i made a new variable (current_shift) in order to not affect the shift variable of the class
        int[] purple = new int[2], order = new int[4];


        for(int i=1;i<=3;i++){
            order[i] = 0;
            if (storageSlots[i].color == COLORS.GREEN) {
                green_pos = i;
            }
            else if (storageSlots[i].color == COLORS.PURPLE)
                purple[cont_purple++] = i;
        }
        current_motif -= current_shift;
        if (current_motif<=0) {
            current_motif = 3+current_motif;
        }
        if (green_pos!=0) order[current_motif] = green_pos;
        cont_purple = 0;
        for(int i=1;i<=3;i++){
            if (i != current_motif ){
                order[i] = purple[cont_purple++];
            }
        }
        return order;
    }

    private void telemetryData(){
        telemetry.addData("Shooting index: ",shooting_index);
        telemetry.addData("Moving state: ",state.val);
        for(int i=1;i<=3;i++){
            telemetry.addData("COLOR: -> Storage "+i+": ",storageSlots[i].color.val);
        }
        //I made two for loops so it would look better
        for(int i=1;i<=3;i++){
            telemetry.addData("Shooting order "+i+": ",shooting_order[i]);
        }
        telemetry.addLine("-----------------------------------------------------"); // separate the mechanisms to make the text easier to read
    }

    
}