package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

public class AutonomousProcedure implements Runnable {
    private ArrayList<ProcedureStep> procedureSteps = new ArrayList<ProcedureStep>();

    private StepStatus lastStepStatus = StepStatus.Waiting;
    private int completedSteps = 0;
    private int runningSteps = 0;
    private int waitingSteps = 0;

    /**
     * Returns true if the procedure is complete.
     */
    public boolean isComplete() {
        return lastStepStatus == StepStatus.Done;
    }

    /**
     * Adds a step that will only run upon the previous's completion. The passed
     * is `StepStatus` may safely be ignored and should always represent a
     * completed step.
     */
    public AutonomousProcedure andThen(Function<StepStatus, StepStatus> fn) {
        procedureSteps.add(new ProcedureStep(fn, true));
        return this;
    }

    /**
     * Adds a step that will run regardless of the previous's completion.
     * It is expected that the given function will be able to manage itself
     * based on the given `StepStatus` of the previous step.
     */
    public AutonomousProcedure and(Function<StepStatus, StepStatus> fn) {
        procedureSteps.add(new ProcedureStep(fn, false));
        return this;
    }

    @Override
    public void run() {
        completedSteps = 0;
        runningSteps = 0;
        completedSteps = 0;
        StepStatus prevState = StepStatus.Done;

        for (ProcedureStep step : procedureSteps) {
            prevState = step.run(prevState);

            if (prevState == StepStatus.Done) {
                completedSteps++;
            }

            if (prevState == StepStatus.Running) {
                runningSteps++;
            }

            if (prevState == StepStatus.Waiting) {
                waitingSteps++;
            }
        }

        lastStepStatus = prevState;
    }

    /**
     * If the given function does not indicate completion by the given time in 
     * seconds, indicate completion anyways.
     */
    public static Function<StepStatus, StepStatus> timeoutAt(
        double timeoutSeconds,
        Function<StepStatus, StepStatus> fn
    ) {
        StatusedTimer timer = new StatusedTimer();

        return (prevStatus) -> {
            if (!timer.getStatus()) {
                timer.start();
            } else if (timer.get() >= timeoutSeconds) {
                return StepStatus.Done;
            }

            return fn.apply(prevStatus);
        };
    }

    /**
     * Step of a procedure, handling the state in which it starts.
     */
    public class ProcedureStep {
        private Function<StepStatus, StepStatus> fn;
        private StepStatus lastState = StepStatus.Waiting;
        private boolean isWaitingStep;

        public ProcedureStep(Function<StepStatus, StepStatus> fn, boolean isWaitingStep) {
            this.fn = fn;
            this.isWaitingStep = isWaitingStep;
        }

        public StepStatus run(StepStatus previousStep) {
            if (isWaitingStep && previousStep != StepStatus.Done) {
                return StepStatus.Waiting;
            }

            if (lastState != StepStatus.Done) {
                lastState = fn.apply(previousStep);
            }
            
            return lastState;
        }
    }

    public enum StepStatus {
        Waiting,
        Running,
        Done
    }

    public void recordState() {
        Logger.recordOutput("Autonomous Procedure Completed", isComplete());
        Logger.recordOutput("Autonomous Procedure Steps Complete", completedSteps);
        Logger.recordOutput("Autonomous Procedure Steps Running", runningSteps);
        Logger.recordOutput("Autonomous Procedure Steps Waiting", waitingSteps);
    }
}
