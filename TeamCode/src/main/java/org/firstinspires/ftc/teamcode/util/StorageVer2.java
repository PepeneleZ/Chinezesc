package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants.COLORS;
import org.firstinspires.ftc.teamcode.util.Constants.MOTIF;
import org.firstinspires.ftc.teamcode.util.Constants.MOVING_STATES;

@Deprecated
public class StorageVer2 implements Updateable {
    public StorageSlot[] storageSlots = new StorageSlot[3]; // am schimbat pana la urma sa fie de la 0 la 2 pt simplicity sake

    public MOTIF motif = MOTIF.GPP; // trb schimvate valorile de la 0-2
    static final COLORS[][] MOTIFS = {
        {COLORS.GREEN, COLORS.PURPLE, COLORS.PURPLE},
        {COLORS.PURPLE, COLORS.GREEN, COLORS.PURPLE},
        {COLORS.PURPLE, COLORS.PURPLE, COLORS.GREEN}
    };
    public MOVING_STATES state = MOVING_STATES.NOTHING;
    //public Intake intake;
    private final ElapsedTime transferTimer;
    private final Telemetry telemetry;
    public int shooting_index = 0;
    public int[] shooting_order;
    public int shift = 0;

    public StorageVer2(HardwareMap hardwareMap, Telemetry telemetry, Intake intake){
        for (int i = 0; i < storageSlots.length; i++){
            storageSlots[i] = new StorageSlot(hardwareMap, i); //trebuie schimbate numele in HardwareConfig
        }
        //this.intake = intake; //nu as controla intake ul de aici
        this.transferTimer = new ElapsedTime();
        this.telemetry = telemetry;
    }

    @Override
    public void update() {
        switch (state){
            case WAITING_INTAKE:
                if(isFull()){
                    state = MOVING_STATES.NOTHING;
                }
                break;

            case SHOOTING:
                updateShooting();
                break;
        }

        for(StorageSlot slot : storageSlots){
            slot.update();
        }
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

    public void toggle_transfer(int index){ //toggle_transfer are mai mult sens pentru ce face codul asta
        storageSlots[index].toggle_transfer(!storageSlots[index].isUP); // sa schimbam numele in storage slots la toggle_servo sau cv
        transferTimer.reset();
        transferTimer.startTime();

//            if(Turret.turret_launcher_state != Constants_Enums.TURRET_LAUNCH_SPEEDS.STOPPED || Turret.velocityPID.targetVelocity>500) {
//                storageSlots[index].color = COLORS.EMPTY;
//            } nici asta nu as controla de aici
    }
    public void toggle_transfer(int index, boolean toggle){ //toggle
        storageSlots[index].toggle_transfer(toggle);
        transferTimer.reset();
        transferTimer.startTime();
    }

    public int[] getShootingOrder(){
        COLORS[] motif = MOTIFS[this.motif.val];
        int remainingShots = 3 - (shift % 3);

        boolean[] used = new boolean[storageSlots.length];
        int[] order = new int[remainingShots];

        int orderIndex = 0;

        for(int i = 0; i < remainingShots; i++){
            COLORS needed = motif[i];

            for(int slot = 0; slot < storageSlots.length; slot++){
                if(!used[slot] && storageSlots[slot].color == needed){
                    order[orderIndex++] = slot;
                    used[slot] = true;
                    break;
                }
            }
        }

        return order;
    }

    public boolean isFull(){
        for(StorageSlot slot : storageSlots){
            if (slot.color == COLORS.EMPTY) {
                return false;
            }
        }
        return true;
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