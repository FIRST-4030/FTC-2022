package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop;

public class MecanumDriveState<E extends MecanumDriveState.Conditional<?>> {

    public interface Conditional<T>{
        boolean isDone();
        void update(Object value);
        String getName();
    }

    public static class TimeCondition implements Conditional<Double>{

        private String name = "TimeCondition";
        private double timeEnd, currentTime;
        public TimeCondition(double end){
            this.timeEnd = end;
            currentTime = 0;
        }

        @Override
        public boolean isDone() {
            return timeEnd <= currentTime && timeEnd != -1;
        }

        @Override
        public void update(Object value) {
            currentTime = (double) value;
        }

        @Override
        public String getName() {
            return name;
        }
    }

    public static class ValueCondition<T extends Comparable> implements Conditional<T>{

        private T endValue, currentValue, acceptable;
        private String name = "ValueCondition: ";
        public ValueCondition(T endValue, T acceptableRange){
            this.endValue = endValue;
            this.name += endValue.getClass().getSimpleName();
        }

        @Override
        public boolean isDone() {
            return endValue.compareTo(currentValue) == 0;
        }

        @Override
        public void update(Object value) {
            currentValue = (T) value;
        }

        @Override
        public String getName() {
            return name;
        }
    }

    public E condition;
    public Runnable state;
    public String name;

    public MecanumDriveState(String name,Runnable state, E condition){
        this.name = name;
        this.condition = condition;
        this.state = state;
    }

    public boolean isDone(){
        return condition.isDone();
    }

    public void update(Object dt){
        condition.update(dt);
    }
}
