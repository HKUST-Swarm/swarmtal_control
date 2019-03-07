//
// Created by xuhao on 3/4/19.
//

#ifndef PROJECT_CONTROLLERS_H
#define PROJECT_CONTROLLERS_H

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
    }

    virtual inline double control(const double & err, double dt, bool report=false) {
        //TODO:
        if (!inited) {
            err_last = err;
            inited = true;
        }

        double ret = err * param.p + err_integrate * param.i + (err - err_last)/dt * param.d;

        err_last = err;
        err_integrate = err_integrate + err * dt;

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

        PIDController::control(err, dt, report);
    }

};


#endif //PROJECT_CONTROLLERS_H
