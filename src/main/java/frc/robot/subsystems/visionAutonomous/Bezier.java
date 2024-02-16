package frc.robot.subsystems.visionAutonomous;

import java.util.List;

public class Bezier {
    int len;
    double[] pointstart;
    double[] pointend;
    List<double[]> listpoint;
    BezierType type;

    public static class Utils{
        public static double[] mult(double[] list, double f){
            double n1 = list[0];
            double n2 = list[1];
            double[] new_list = {n1*f,n2*f};
            return new_list;
        }
        public static double[] add(double[] l1, double[] l2){
            double[] new_list = {l1[0]+l2[0], l1[1]+l2[1]};
            return new_list;
        }
        public static double[] add(double[] l1, double[] l2, double[] l3){
            double[] new_list = {l1[0]+l2[0]+l3[0], l1[1]+l2[1]+l3[1]};
            return new_list;
        }
        public static double[] sub(double[] l1, double[] l2){
            double[] new_list = {l1[0]-l2[0], l1[1]-l2[1]};
            return new_list;
        }
        public static double sqrt(double value){
            return value*value;
        }
    }

    private enum BezierType{
        Lin,
        Quad
    }

    public Bezier(double[] startPos, List<double[]> pos, double[] endPos){
        len = pos.size();
        pointstart = startPos;
        pointend = endPos;
        listpoint = pos;
        if (len == 0){
            type = BezierType.Lin;
        }
        if (len == 1){
            type = BezierType.Quad;
        }
    }

    private double[] get_value_lin(double t){
        double[] point = Utils.add(Utils.mult(pointstart,(1-t)),Utils.mult(pointend,t));
        return point;
    }

    private double[] get_value_quad(double t){
        double[] point = Utils.add(
        Utils.mult(pointstart,Utils.sqrt(1-t)), 
        Utils.mult(listpoint.get(0),2*t*(1-t)),
        Utils.mult(pointend, Utils.sqrt(t)));
        return point;
    }

    public double[] get_div(double t, double dt){
        double[] point1 = get_value(t);
        double[] point2 = get_value(t + dt);
        double[] richtung = Utils.sub(point1, point2);
        return richtung;
    }

    public double[] get_value(double t){
        if (type == BezierType.Lin){
            return get_value_lin(t);
        } else if (type == BezierType.Quad){
            return get_value_quad(t);
        }else{
            double[] val = {};
            System.out.println("error bezier");
            return val;
        }
    }
}
