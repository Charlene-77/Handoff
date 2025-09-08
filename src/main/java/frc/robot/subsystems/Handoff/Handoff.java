package frc.robot.subsystems.Handoff;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Handoff extends SubsystemBase {
    private final HandoffIO handoffIO;
    private HandoffIOInputsAutoLogged inputs = new HandoffIOInputsAutoLogged();
    private HandoffStates handoffState = HandoffStates.IDLE;
    private double setpointVolts;

    public enum HandoffStates{
        IDLE,
        RUN
    }

    public Handoff(HandoffIO handoffIO) {
        this.handoffIO = handoffIO;
        setpointVolts = 0.0;
    }


    @Override
    public void periodic(){
        handoffIO.updateInputs(inputs);
        Logger.processInputs("Handoff", inputs);
        Logger.recordOutput("HandoffState", handoffState);
        switch(handoffState){
            case IDLE:
                handoffIO.runHandoff(0);
                break;
            case RUN:
                handoffIO.runHandoff(10);
                break;
            default:
                break;
        }
    }

    public void runHandoff(double voltage) {
        setpointVolts = voltage;
        handoffIO.runHandoff(setpointVolts);
    }

    public double getStatorCurrent() {
        return inputs.currentAmps;
    }

    public void updateInputs(HandoffIO.HandoffIOInputs inputs) {
        handoffIO.updateInputs(inputs);
    }
}

//hi
