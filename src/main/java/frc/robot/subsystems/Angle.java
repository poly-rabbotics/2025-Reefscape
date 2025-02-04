// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public final class Angle implements Cloneable {
    public static final double TAU = Math.PI * 2;

    private double dec = 0.0;

    public double radians() {
        return dec * TAU;
    }

    public double degrees() {
        return dec * 360.0;
    }

    public Angle setRadians(double radians) {
        dec = radians / TAU;
        return this;
    }

    public Angle setDegrees(double degrees) {
        dec = degrees / 360.0;
        return this;
    }

    public Angle add(Angle other) {
        var angle = new Angle();
        angle.dec = this.dec + other.dec;
        return angle;
    }

    public Angle sub(Angle other) {
        var angle = new Angle();
        angle.dec = this.dec - other.dec;
        return angle;
    }

    public Angle mul(double scalar) {
        var angle = new Angle();
        angle.dec = this.dec * scalar;
        return angle;
    }

    public Angle div(double scalar) {
        return this.mul(1 / scalar);
    }

    public Angle bound180() {
        Angle ret = new Angle().setDegrees((degrees() % 360.0 + 360.0) % 360.0);

        if (ret.degrees() > 180.0) {
            ret = ret.setDegrees(ret.degrees() - 360.0);
        } else {
            ret = ret.setDegrees(degrees());
        }


        assert ret.degrees() <= 180.0;
        assert ret.degrees() > -180.0;

        System.out.println(ret.degrees());

        return ret;
    }

    public Angle clamp(Angle a, Angle b) {
        if (dec < a.dec && dec < b.dec) {
            return a.clone();
        } else if (dec > b.dec && dec > a.dec) {
            return b.clone();
        } else {
            return this.clone();
        }
    }

    @Override
    public Angle clone() {
        var ang = new Angle();
        ang.dec = this.dec;
        return ang;
    }

    @Override
    public String toString() {
        return this.degrees() + " degrees";
    }
}
