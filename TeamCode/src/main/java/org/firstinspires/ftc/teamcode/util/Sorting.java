package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.MultiplePIDF;
import org.firstinspires.ftc.teamcode.lib.tests.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.Constants.COLORS;
import org.firstinspires.ftc.teamcode.util.Constants.INTAKE_STATES;
import org.firstinspires.ftc.teamcode.util.Constants.MOVING_STATES;
import org.firstinspires.ftc.teamcode.util.Constants.TRANSFER_POS;
import org.firstinspires.ftc.teamcode.util.Constants.TURRET_LAUNCH_SPEEDS;
import org.firstinspires.ftc.teamcode.util.Constants.MOTIF;



//     (. Y .)
//       ).(
//      \ Y /
//       | |


@Config
public class Sorting implements  Updateable{
    //---------MOTORS--------------------
    public Servo transfer_servo;
    public DcMotorEx blade;
    public ColorSensor color_sensor_intake;
    private final Intake intake;
    private final VoltageSensor voltageSensor;
    //public static PIDF pidElice = new PIDF(0.000163d,0.0000062d,0.00000091d);
    public static MultiplePIDF pidElice = new MultiplePIDF(0.00198,0.0002,2e-5d);

    private final Telemetry telemetry;


    //----------POSITIONS-------------
    public TRANSFER_POS transfer_pos = TRANSFER_POS.DOWN;
    public static final double full_rotation = 384.5;
    public static final double one_rotation = full_rotation/6.0;
    private static final double durationOfOneRotation = 0.5;
    private double slightlyMovingDuration=0;
    public double position=0; // increases or decreses forever;
    public int blade_rotation=0; // between [0, full_rotation] equal to position % full_rotation
    public int current_blade_index=0;
    public int last_blade_index=0;
    public int shift=0;
    public int shooting_index=1;
    public double target;
    public double pidError;
    private final ElapsedTime transferTimer, collectTimer, slightlyMovingTimer;
    private boolean transfer_isUp = false;
    private int shooting_balls = 0;
    private static final int admissible_error=7;
    private boolean runPid=false;
    private double manualDeviation = 0;
    private double deltaManualDeviation = 0;
    private double startOfManualDeviation = 0;
    public boolean respectMotif = true;
    public static double kF_fornrofballs=0;
    public double collectTime = 0;
    private double rotations_for_telemetry;

    //momentul de frecare e momentu care se opune momentului motorului (care misca elicea)
    //
    //----------STATES-----------------
    public static COLORS[] magazine = new COLORS[7];// 2 - intake || 4 - aruncare  || 6 - rezerva   1 - intrare human
    public static COLORS[] shooting_order = new COLORS[4];
    public static MOTIF motif = MOTIF.GPP;
    public MOVING_STATES current_moving_state = MOVING_STATES.NOTHING;
    public MOVING_STATES next_moving_state = MOVING_STATES.NOTHING;
    public MOVING_STATES last_moving_state = MOVING_STATES.NOTHING;



    public Sorting(HardwareMap hwmap, Telemetry telemetry, Intake intake, VoltageSensor voltageSensor){
        blade = hwmap.get(DcMotorEx.class, HardwareConfig.sorting);
        transfer_servo = hwmap.get(Servo.class,HardwareConfig.transfer);
        color_sensor_intake = hwmap.get(ColorSensor.class, HardwareConfig.color_sensor_intake);
        this.voltageSensor = voltageSensor;
        this.intake = intake;
        this.telemetry = telemetry;


        pidElice.addPidCoefficients(0.00198,0.0002,2e-5d);// pentru o rotatie jumate (3/6
        pidElice.addPidCoefficients(0.00198,0.0002,2e-5d);// pentru jumate de rotatie (1/6)



        transferTimer = new ElapsedTime();
        collectTimer = new ElapsedTime();
        slightlyMovingTimer = new ElapsedTime();
        transfer_servo.setDirection(Servo.Direction.REVERSE);

        transfer_servo.setPosition(transfer_pos.val);

        blade.setDirection(DcMotorSimple.Direction.REVERSE);
        blade.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        blade.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        blade.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // encoder_elice.setDirection(DcMotorSimple.Direction.REVERSE);


        pidElice.resetPid();
        clearMagazine();

    }



    public void setNextState(MOVING_STATES state) {
        //        if(state == MOVING_STATES.SHOOTING && Turret.turret_launcher_state == Turret.TURRET_LAUNCH_SPEEDS.STOPPED)
        //            return;
        if (current_moving_state == MOVING_STATES.MOVING) {
            next_moving_state = state;
            return;
        }
        else current_moving_state = state;


        if (state == MOVING_STATES.WAITING_INTAKE) {
            collectTimer.reset();
            collectTimer.startTime();
            collectTime = 0;

//              if (current_blade_index % 2 == 1)
//                  rotate_elice(0.5);
        }
        else if (state == MOVING_STATES.SHOOTING){
            if (current_blade_index % 2 == 0)
                rotate_elice(0.5);
//            if(Math.abs(pidError)<admissible_error*1.1){ // se poate schimba valorea
//                current_moving_state = MOVING_STATES.NOTHING;
//                last_moving_state = MOVING_STATES.NOTHING;
//            }
            shooting_balls = getNumberOfBalls();
            shooting_order = getShooting_order();
            shooting_index = 1;
            if (shooting_balls == 0)
                current_moving_state = MOVING_STATES.NOTHING;

        }
    }


    public void manualBlade(){
        if (startOfManualDeviation==0) startOfManualDeviation = position;
        else deltaManualDeviation = position-startOfManualDeviation;
        if (deltaManualDeviation>=one_rotation){
            target += one_rotation;
            deltaManualDeviation-=one_rotation;
        }
        pidElice.setTargetPosition(position);
        runPid = false;
        blade.setPower(manualDeviation);
    }
    public void resetStates(){
        current_moving_state = MOVING_STATES.NOTHING;
        next_moving_state = MOVING_STATES.NOTHING;
        pidElice.resetPid();
    }
    public void resetManual(){
        manualDeviation = 0;
        deltaManualDeviation = 0;
        startOfManualDeviation = 0;
    }
    public void resetEverything(){
        resetStates();
        pidElice.setTargetPosition(0);
        pidElice.resetPid();
        last_moving_state = MOVING_STATES.NOTHING;
        last_blade_index = 0;
        current_blade_index = 0;
        //isManually = false;
        resetManual();
        runPid = false;
        transfer_ball(false);
        transfer_isUp = false;
        clearMagazine();
        pidError = 0;

        position = 0;
        blade_rotation = 0;
        target = 0;
        shooting_balls = 0;
        target = 0;
        pidError = 0;
        intake.toggle(INTAKE_STATES.STOPPED);
        Turret.setTarget_rotation(TURRET_LAUNCH_SPEEDS.STOPPED);

        blade.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        blade.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        blade.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //encoder_elice.setDirection(DcMotorSimple.Direction.REVERSE);



    }
    public void transfer_ball(){ //toggle
        if (current_blade_index % 2 == 0) {
            return;
        }
        if(!transfer_isUp){
            transfer_servo.setPosition(TRANSFER_POS.UP.val);

            transferTimer.reset();
            transferTimer.startTime();

            if(Turret.turret_launcher_state != TURRET_LAUNCH_SPEEDS.STOPPED || Turret.feedforwardController.targetVelocity>500) {
                magazine[5] = COLORS.EMPTY;
            }
        }
        else{
            transfer_servo.setPosition(TRANSFER_POS.DOWN.val);
            transferTimer.reset();
        }
        transfer_isUp = !transfer_isUp;

    }
    public void transfer_ball(boolean direction){ // true = up, false = down
        if (current_blade_index % 2 == 0) {
            return;
        }
        if(direction){
            transfer_servo.setPosition(TRANSFER_POS.UP.val);
            transferTimer.reset();
            transferTimer.startTime();
            transfer_isUp = true;

            if(Turret.turret_launcher_state != TURRET_LAUNCH_SPEEDS.STOPPED || Turret.feedforwardController.targetVelocity>500) {
                magazine[5] = COLORS.EMPTY;
            }
        }
        else{
            transfer_servo.setPosition(TRANSFER_POS.DOWN.val);
            transferTimer.reset();
            transfer_isUp = false;
        }
    }

    public void rotate_elice(double turns){ // positive is right, negative is left
        if(turns==0) return;
        if (getNumberOfBalls()==0 && (int)turns!=turns)
            pidElice.switchPid(4);
        else
            pidElice.switchPid(0);
        //pidElice.switchPid(getNumberOfBalls());

        last_moving_state = current_moving_state;

        current_moving_state = MOVING_STATES.MOVING;
        turns = turns % 4; //o sa ai valori cuprinse intre -3 si 3
        //target = (int)(position / full_rotation) * full_rotation + full_rotation/6 * pos_to_index((int)position) + one_rotation*turns*2;
        target += turns*one_rotation*2.0;
        pidElice.resetPid();
        pidElice.setTargetPosition(target);
        runPid = true;

        // sa se invarta usurel intake-ul
        intake.toggle(INTAKE_STATES.SLIGHTLY_MOVING);
        slightlyMovingTimer.reset();
        slightlyMovingTimer.startTime();
        slightlyMovingDuration = durationOfOneRotation*turns;
    }

    @Override
    public void update(){
        position = blade.getCurrentPosition();
        blade_rotation = (int)position % (int)full_rotation;
        if(blade_rotation<0)blade_rotation=(int)full_rotation+blade_rotation;
        pidError = target-position;

        if (runPid) {
            double power;
            power = pidElice.update(position) * (14/voltageSensor.getVoltage()) + kF_fornrofballs*getNumberOfBalls();
            //            power = pidElice.update(position) * (14/voltageSensor.getVoltage());
            blade.setPower(power);
        }
        else if ((manualDeviation>0?manualDeviation:-manualDeviation)>0.2) {
            manualBlade();
        }
        else blade.setPower(0);

        if (intake.intake_state == INTAKE_STATES.SLIGHTLY_MOVING && (current_moving_state != MOVING_STATES.MOVING || slightlyMovingTimer.seconds() >slightlyMovingDuration)){
            if (intake.fromCollectingToSlightly)
                intake.toggle(INTAKE_STATES.COLLECTING);
            else
                intake.toggle(INTAKE_STATES.STOPPED);
            slightlyMovingTimer.reset();
        }

        if (Math.abs(pidError) > admissible_error*3 && !runPid && Turret.turret_launcher_state == TURRET_LAUNCH_SPEEDS.STOPPED) {
            runPid = true;
            pidElice.resetPid();
        }

        if(transfer_isUp && transferTimer.milliseconds()>200){
            transfer_ball(false);
        }

        current_blade_index = pos_to_index((int)position);

        if (current_blade_index != last_blade_index){
            boolean direction_of_rotation;
            if (current_blade_index==6&&last_blade_index==1)
                direction_of_rotation = false;
            else if (current_blade_index==1&&last_blade_index==6)
                direction_of_rotation = true;
            else
                direction_of_rotation =(current_blade_index-last_blade_index>0);

            rotate_balls(direction_of_rotation);
        }
        last_blade_index = current_blade_index;


        if (current_moving_state == MOVING_STATES.NOTHING || current_moving_state == MOVING_STATES.WAITING_INTAKE&& next_moving_state != MOVING_STATES.NOTHING){
            last_moving_state = current_moving_state;
            current_moving_state = next_moving_state;
            next_moving_state = MOVING_STATES.NOTHING;
            return;
        }
        //MMMMMOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOVIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIINGGGGGGGGGGGGGGGGGGGGGG
        if (current_moving_state == MOVING_STATES.MOVING){

            if ((pidError>0?pidError:-pidError) < admissible_error && runPid && !((manualDeviation>0?manualDeviation:-manualDeviation)>0.2)){
                //if ((position>0?position:-position)>(target>0?target:-target)){
                pidElice.resetPid();
                runPid = false;
                if (last_moving_state==MOVING_STATES.WAITING_INTAKE || last_moving_state==MOVING_STATES.SHOOTING) {
                    current_moving_state = last_moving_state;
                    resetManual();
                    return;
                }
                current_moving_state = MOVING_STATES.NOTHING;

            }
        }

        //IIIIIIIIIIIINNNNNNNNNNNNNNNNNNNNNNTTTTTTTTTTTTTTTTTTTAAAAAAAAAAAAAAAAAAKKKKKKKKKKKKKKKEEEEEEEEEEE
        if (current_moving_state == MOVING_STATES.WAITING_INTAKE){
            if (isFull()){
                last_moving_state = MOVING_STATES.NOTHING;
                current_moving_state = MOVING_STATES.NOTHING;
                return;
            }
            if (current_blade_index % 2 == 1) {
                if (magazine[1] == COLORS.EMPTY) rotate_elice(0.5);
                else rotate_elice(-0.5);

            }
            COLORS current_color = ColorFunctions.getColor(color_sensor_intake.red(),color_sensor_intake.green(),color_sensor_intake.blue());
            if (current_color != COLORS.EMPTY){
                if(magazine[2] != COLORS.EMPTY){
                    setNextState(MOVING_STATES.NOTHING);
                    rotate_elice(1);

                    return;
                }

                magazine[2] = current_color;
                if (magazine[6] == COLORS.EMPTY)
                    rotate_elice(1);
                else if (magazine[4] == COLORS.EMPTY)
                    rotate_elice(-1);
            }
        }
        //SHOOOOOOOOOOOOOOOOOOOOOOOOOOOOOTIIIIIIIIIIIIIIIIIIIIIINNNNNNNNNNNNNNNNNNNGGGGGGGGG
        if (current_moving_state == MOVING_STATES.SHOOTING){
            if(transfer_isUp || transferTimer.milliseconds()<200)
                return;
            //if(shooting_balls<=0){
            if(shooting_index>3){
                exit_shooting();
                return;
            }
            else if (shooting_order[shooting_index]==COLORS.EMPTY){
                exit_shooting();
                return;
            }

            if (magazine[5]==shooting_order[shooting_index]) {
                shooting_index++;
                shooting_balls--;
                transfer_ball(true);
            }
            else if (magazine[1]==shooting_order[shooting_index]) {
                rotate_elice(-1);
                return;
            }
            else if (magazine[3]==shooting_order[shooting_index]){
                rotate_elice(1);
                return;
            }
            else if (!respectMotif) exit_shooting();

        }
        resetManual();
    }
    public void exit_shooting(){
        last_moving_state = MOVING_STATES.NOTHING;
        current_moving_state = MOVING_STATES.NOTHING;
        rotate_elice(-0.5);
        Turret.setTarget_rotation(TURRET_LAUNCH_SPEEDS.STOPPED);
        //clearMagazine();
    }
    public void sort_balls(){
        int current_green_pos = getGreen();
        double rotations=0;
        // int rotations = motif - current_green_pos;
        int nrofballs = getNumberOfBalls();

        if (nrofballs==0) return;
        if(nrofballs==1){
            for(int i=1;i<=6;i++){
                if(magazine[i] != COLORS.EMPTY) {
                    rotations = (4 - i)/2.0;
                }
            }
        }
        else if (nrofballs==2){
            if (current_blade_index % 2 ==0) {
                if (magazine[2] != COLORS.EMPTY && magazine[6] != COLORS.EMPTY)
                    rotations = -1;
                else if (magazine[4] != COLORS.EMPTY && magazine[2] != COLORS.EMPTY)
                    rotations = 1;
                else if (magazine[4] != COLORS.EMPTY && magazine[6] != COLORS.EMPTY)
                    rotations = 0;
            }
            else {
                if (magazine[1] != COLORS.EMPTY && magazine[3] != COLORS.EMPTY)
                    rotations = 1.5;
                else if (magazine[1] != COLORS.EMPTY && magazine[5] != COLORS.EMPTY)
                    rotations = -0.5;
                else if (magazine[3] != COLORS.EMPTY && magazine[5] != COLORS.EMPTY)
                    rotations = 0.5;
            }
        }
        else if (nrofballs == 3){
            rotations = (motif.val - current_green_pos)/2.0;
        }
        rotations_for_telemetry = rotations;


        rotate_elice(rotations);

    }
    public COLORS[] getShooting_order(){
        int current_shift = shift%4, current_motif= motif.val;
        COLORS[] order = new COLORS[4];
        current_motif -= shift;
        if (current_motif<=0) {
            current_motif = 3+current_motif;
        }
        for(int i=1;i<=3;i++){
            if (motif.val==i) order[i] = COLORS.GREEN;
            else order[i] = COLORS.PURPLE;
            if (i>getNumberOfBalls()) order[i] = COLORS.EMPTY;
        }
        return order;


    }

    public static int pos_to_index(int position){
        if (position<0) position = -position;
        position %= (int)full_rotation;

        int index=0, min_dif=10000000, dif=0;
        for (int i=0;i<=6;i++){
            dif = position-(int)full_rotation/6*i;
            if(dif<0)dif=-dif;

            if (dif<min_dif){
                min_dif = dif;
                index = i;
            }
        }
        index+=2;
        if (index>6) index %=6;
        return index;
    }

    public static void rotate_balls(boolean right){
        COLORS aux;
        if (right){
            for(int i=6;i>=2;i--){
                aux = magazine[i];
                magazine[i] = magazine[i-1];
                magazine[i-1] = aux;
            }
        }
        else {
            for(int i=2;i<=6;i++){
                aux = magazine[i-1];
                magazine[i-1] = magazine[i];
                magazine[i] = aux;
            }

        }

    }
    public boolean isFull(){
        if((magazine[2] != COLORS.EMPTY && magazine[4] != COLORS.EMPTY && magazine[6] != COLORS.EMPTY) || (magazine[1] != COLORS.EMPTY && magazine[3] != COLORS.EMPTY && magazine[5] != COLORS.EMPTY)){
            intake.toggle(INTAKE_STATES.STOPPED);
            if (collectTime==0) {
                collectTime = collectTimer.seconds();
                collectTimer.reset();
            }
            return true;
        } else {
            return false;
        }
    }
    public static int getGreen(){
        for(int i=1;i<=6;i++){
            if (magazine[i]==COLORS.GREEN)
                return i;
        }
        return 0;
    }
    public int getNumberOfBalls(){
        int nrofballs=0;
        if (magazine[1]!=COLORS.EMPTY || magazine[3]!=COLORS.EMPTY || magazine[5]!=COLORS.EMPTY){
            magazine[2] = COLORS.EMPTY;
            magazine[4] = COLORS.EMPTY;
            magazine[6] = COLORS.EMPTY;
        }
        else if (magazine[2] != COLORS.EMPTY || magazine[4] != COLORS.EMPTY || magazine[6] != COLORS.EMPTY) {
            magazine[1] = COLORS.EMPTY;
            magazine[3] = COLORS.EMPTY;
            magazine[5] = COLORS.EMPTY;
        }
        for (int i=1;i<=6;i++){
            if (magazine[i]!=COLORS.EMPTY) nrofballs++;
        }
          /* alternative
          int starting_i;

          if (current_blade_index%2==0) starting_i = 2;
          else starting_i = 1;

          for(int i=starting_i;i<=6;i+=2){
              if (magazine[i] != COLORS.EMPTY) {
                  rotations = 4-i;
                  return;
              }
          }
          */
        return nrofballs;

    }
    public void clearMagazine(){
        //        magazine[1] = COLORS.EMPTY;
        //        magazine[2] = COLORS.GREEN;
        //        magazine[3] = COLORS.EMPTY;
        //        magazine[4] = COLORS.PURPLE;
        //        magazine[5] = COLORS.EMPTY;
        //        magazine[6] = COLORS.PURPLE;
        for(int i=1;i<=6;i++){
            magazine[i] = COLORS.EMPTY;
        }
    }
    public void fillMagazine(){
        magazine[1] = COLORS.EMPTY;
        magazine[2] = COLORS.GREEN;
        magazine[3] = COLORS.EMPTY;
        magazine[4] = COLORS.PURPLE;
        magazine[5] = COLORS.EMPTY;
        magazine[6] = COLORS.PURPLE;
    }



    public  void telemetryData(){
        telemetry.addData("Absolute bladep pos: ",position);
        telemetry.addData("Moded blade pos: ",blade_rotation);
        telemetry.addData("STATEEEE", current_moving_state.val);
        telemetry.addData("Number of balls: ",getNumberOfBalls());
        telemetry.addData("MOTIFFFF", motif);

        telemetry.addData("Current_blade_index", current_blade_index);
        telemetry.addData("Pos_to_index",pos_to_index((int)position));
        telemetry.addData("blade_rotation % one_rotation",blade_rotation%one_rotation);
        telemetry.addData("Target: ",target);
        telemetry.addData("Pid error",pidError);
        telemetry.addData("Timer: ",transferTimer.milliseconds());
        telemetry.addData("runPid: ",runPid);
        telemetry.addData("Which pid: ", pidElice.whichPid());
        telemetry.addData("Collecting time: ",collectTime);
        telemetry.addData("Rotations for sorting: ",rotations_for_telemetry);

        telemetry.addData("1: ", magazine[1].val);
        telemetry.addData("2: ", magazine[2].val);
        telemetry.addData("3: ", magazine[3].val);
        telemetry.addData("4: ", magazine[4].val);
        telemetry.addData("5: ", magazine[5].val);
        telemetry.addData("6: ", magazine[6].val);

    }


}