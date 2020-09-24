//
// Created by xuhao on 3/4/19.
//

#ifndef PROJECT_CONTROLLERS_H
#define PROJECT_CONTROLLERS_H

#define THRES_RY 0.1

inline double lowpass_filter(double input, double Ts, double outputLast, double dt) {
    double alpha = dt / (Ts + dt);
    return outputLast + (alpha * (input - outputLast));
}

inline double float_constrain(double v, double min, double max)
{
    if (v < min) {
        return min;
    }
    if (v > max) {
        return max;
    }
    return v;
}



struct PIDParam {
    double p = 0;
    double i = 0;
    double d = 0;
    double b = 1.0;
    double c = 1.0;
    double tf = 0.01;
    double max_err_i = 0;
};

struct SchulingPIDParam {
    std::vector<double> v_list;
    std::vector<double> p;
    std::vector<double> i;
    std::vector<double> d;
    double max_err_i = 0;

    int cases_num()
    {
        return v_list.size();
    }

};

class PIDController {
    double err_integrate = 0;
    double err_last = 0;
    bool inited = false;
    double offset = 0;
    double err_d_filtered = 0;
protected:
    PIDParam param;

public:
    PIDController(PIDParam _param):
            param(_param)
    {}

    PIDController()
    {}

    void reset() {
        err_integrate = 0;
        err_last = 0;
        err_d_filtered = 0;
        inited = false;
    }

    virtual inline double control2(const double & r, const double & y , const double & dt, bool report=false) {
        if (!inited) {
            err_last = (param.c*r-y);
            err_d_filtered = 0;
            inited = true;
                
	        if (fabs(r-y) < THRES_RY && fabs(param.b - 1.0) > 0.1) {
	    	    offset =  -(param.b*r-y)*param.p;
                printf("PID2 Controller INIT R %f Y %f OFFset %f\n", r, y, offset);
	        }
        }

        err_integrate = err_integrate + (r-y) * dt;

        err_d_filtered = lowpass_filter(((param.c*r-y) - err_last)/dt, param.tf, err_d_filtered, dt);

        double ret = (param.b*r - y) * param.p + err_integrate * param.i + err_d_filtered * param.d + offset;

        err_last = param.c*r - y;

        if (err_integrate > param.max_err_i)
        {
            err_integrate = param.max_err_i;
        }
        if (err_integrate < - param.max_err_i)
        {
            err_integrate = -param.max_err_i;
        }

        if (report) {
            printf("DT[%f] R %f Y %f ERR %3.2f ERRI %3.2f OUTPUTI %3.2f ERRD_F %3.2f OUTPUTD %3.2f OFFSET %3.2f OUTPUT %3.2f\n",
                dt, r, y, (param.b*r - y) , err_integrate,  err_integrate * param.i, 
                    err_d_filtered, err_d_filtered * param.d, offset,  ret);
        }

        return ret;
 
    }

    virtual inline double control(const double & err, double dt, bool report=false) {
        if (!inited) {
            err_last = err;
            inited = true;
        }

        err_integrate = err_integrate + err * dt;

        double ret = err * param.p + err_integrate * param.i + (err - err_last)/dt * param.d;

        err_last = err;

        if (err_integrate > param.max_err_i)
        {
            err_integrate = param.max_err_i;
        }
        if (err_integrate < - param.max_err_i)
        {
            err_integrate = -param.max_err_i;
        }

        if (report)
         printf("ERR %3.2f ERRI %3.2f OUTPUTI %3.2f OUTPUT %3.2f\n", err, err_integrate,  err_integrate * param.i, ret);

        return ret;


    }
};

class SchulingPIDController: public PIDController {
    SchulingPIDParam schul_param;

public:
    SchulingPIDController(SchulingPIDParam _param):
            schul_param(_param)
    {
        printf("schul size %ld", schul_param.v_list.size());
        param.p = schul_param.p[0];
        param.i = schul_param.i[0];
        param.d = schul_param.d[0];
    }

    void calc_pid(const double x)
    {
        int ptr = 0;
        // printf("Schul cases %d\n", schul_param.cases_num());
        if (schul_param.cases_num() == 1)
        {
            param.p = schul_param.p[0];
            param.i = schul_param.i[0];
            param.d = schul_param.d[0];
            return;
        }

        while (ptr < schul_param.cases_num() && schul_param.v_list[ptr] < x) {
            ptr ++;
        }

        printf("ptr is %d\n", ptr);

        if (ptr == 0) {
            param.p = schul_param.p[0];
            param.i = schul_param.i[0];
            param.d = schul_param.d[0];
            return;
        }

        if (ptr == schul_param.cases_num())
        {
            param.p = schul_param.p[schul_param.cases_num()-1];
            param.i = schul_param.i[schul_param.cases_num()-1];
            param.d = schul_param.d[schul_param.cases_num()-1];
            return;
        }

        //Mix ptr - 1 and ptr

        double d = schul_param.v_list[ptr] - schul_param.v_list[ptr-1];
        double r1 = x - schul_param.v_list[ptr - 1];

        param.p = float_constrain((d-r1)/d * schul_param.p[ptr-1] + r1/d * schul_param.p[ptr], 0, 100);
        param.i = float_constrain((d-r1)/d * schul_param.i[ptr-1] + r1/d * schul_param.i[ptr], 0, 100);
        param.d = float_constrain((d-r1)/d * schul_param.d[ptr-1] + r1/d * schul_param.d[ptr], 0, 100);

        // printf("Lower case is %f P %f, Upper is %f P %f\n", schul_param.v_list[ptr - 1], schul_param.p[ptr - 1], schul_param.v_list[ptr], schul_param.p[ptr]);
        // printf("x %f r1 %f d %f p final %f\n", x, r1, d, param.p);
    }

    virtual inline double control(const double x, const double & err, double dt, bool report=false) {
        this->calc_pid(x);
        // printf("V is %2.1f, PID %3.1f %3.1f %3.1f\n", x, param.p, param.i, param.d);
        return PIDController::control(err, dt, report);
    }

};


#endif //PROJECT_CONTROLLERS_H
