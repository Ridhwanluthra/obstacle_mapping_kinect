package ducic.plumbum.com.eyic2017.models;

/**
 * Created by pankaj on 12/3/17.
 */

public class RosTopic {
    private Double minDistance, leftmost, rightmost;
    private Double angleLeft, angleRight, angleMin;

    public RosTopic(){

    }

    public RosTopic (Double minDistance, Double leftmost, Double rightmost, Double angleLeft, Double angleRight, Double angleMin){
        this.minDistance = minDistance;
        this.leftmost = leftmost;
        this.rightmost = rightmost;
        this.angleLeft = angleLeft;
        this.angleRight = angleRight;
        this.angleMin = angleMin;
    }

    public Double getAngleLeft() {
        return angleLeft;
    }

    public Double getAngleRight() {
        return angleRight;
    }

    public Double getAngleMin() {
        return angleMin;
    }

    public Double getLeftmost() {
        return leftmost;
    }

    public Double getMinDistance() {
        return minDistance;
    }

    public Double getRightmost() {
        return rightmost;
    }
}
