/*
 * NumericalTool.h
 *
 *  Created on: 2021. 7. 1.
 *      Author: KimKyungHwan
 */

#ifndef NUMERICALTOOL_H_
#define NUMERICALTOOL_H_

class NumericalTool{
private:
public:
        NumericalTool();
        virtual ~NumericalTool();

        class SimpleMovingAverage{
        private:
            int LEN;
            double x[2000];
            double y;
            double y_old;
            double avg;
            void Shift(double value_in);
        public:
            SimpleMovingAverage(int len=2000);
            virtual ~SimpleMovingAverage();
            double Filter(double value_in,int len=3);
            void Initialize(double value);

        };

        class Calculus{
        private:
            double dt,inv_dt;
            double x,x_old,x_old2;
            char init_plag;
            double xdot,xddot;
            double Sx;

        public:
            Calculus(double _dt=0.005);
            double Diff(const double& _x);
            double Inte(const double& _x);
            void SetDiffnInte(const double& _x);
            double GetDiff(){return xdot;}
            double GetInte(){return Sx;}
            void Reset(){init_plag=0;Sx=0;}
        };

        class LowPassFilter{
        private:
            double output;
            double pre_output;
            char init_plag;
            double pi2;
        public:
            LowPassFilter():init_plag(0){pi2=6.28318530718;}
            double Filter(double input, double f_c,double t_s=0.005);
        };


};
#endif


