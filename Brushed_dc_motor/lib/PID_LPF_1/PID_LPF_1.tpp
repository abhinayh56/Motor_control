#include "PID_LPF_1.h"

template <typename T>
control_system::pid::PID_LPF_1<T>::PID_LPF_1()
{
    dt = 0.0;
    Kp = 0.0;
    Ki = 0.0;
    Kd = 0.0;
    u_k_1 = 0.0;
    u_k_2 = 0.0;
    e_k_1 = 0.0;
    e_k_2 = 0.0;
    start = 0;
}

template <typename T>
void control_system::pid::PID_LPF_1<T>::init(T dt_, T Kp_, T Ki_, T Kd_, T fc_, T u_k_1_, T u_k_2_, T u_max_)
{
    set_param(dt_, Kp_, Ki_, Kd_, fc_, u_k_1_, u_k_2_, u_max_);
    start = 0;
}

template <typename T>
void control_system::pid::PID_LPF_1<T>::set_param(T dt_, T Kp_, T Ki_, T Kd_, T fc_, T u_k_1_, T u_k_2_, T u_max_)
{
    dt = dt_;
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    tau = 1.0 / (6.28318530717958647692528676655900576 * fc_);
    u_k_1 = u_k_1_;
    u_k_2 = u_k_2_;
    u_max = u_max_;
}

template <typename T>
T control_system::pid::PID_LPF_1<T>::update(T x_0, T x)
{
    T e_k = x_0 - x;
    T u_k = 0.0;
    if ((start == 0) || (start == 1))
    {
        start++;
        u_k = u_k_1 + Kp * e_k;
    }
    else
    {
        u_k = (-tau * u_k_2 + (2.0 * tau + dt) * u_k_1 + ((Kp + Ki * dt) * (dt + tau) + Kd) * e_k - (Kp * (2.0 * tau + dt) + Ki * dt * tau + 2.0 * Kd) * e_k_1 + (tau + Kd) * e_k_2) / (dt + tau);
    }
    u_k = saturate(u_k, -u_max, u_max);
    u_k_2 = u_k_1;
    u_k_1 = u_k;
    e_k_2 = e_k_1;
    e_k_1 = e_k;
    return u_k;
}

template <typename T>
void control_system::pid::PID_LPF_1<T>::reset()
{
    u_k_1 = 0.0;
    u_k_2 = 0.0;
    e_k_1 = 0.0;
    e_k_2 = 0.0;
    start = 0;
}

template <typename T>
void control_system::pid::PID_LPF_1<T>::merge(T u_k_2_)
{
    u_k_2 = u_k_2_;
}

template <typename T>
void control_system::pid::PID_LPF_1<T>::set_dt(T dt_)
{
    dt = dt_;
}

template <typename T>
void control_system::pid::PID_LPF_1<T>::set_Kp(T Kp_)
{
    Kp = Kp_;
}

template <typename T>
void control_system::pid::PID_LPF_1<T>::set_Ki(T Ki_)
{
    Ki = Ki_;
}

template <typename T>
void control_system::pid::PID_LPF_1<T>::set_Kd(T Kd_)
{
    Kd = Kd_;
}

template <typename T>
void control_system::pid::PID_LPF_1<T>::set_fc(T fc_)
{
    tau = 1.0 / (6.28318530717958647692528676655900576 * fc_);
}

template <typename T>
void control_system::pid::PID_LPF_1<T>::set_u_0(T u_k_1_)
{
    u_k_1 = u_k_1_;
}

template <typename T>
void control_system::pid::PID_LPF_1<T>::set_u_max(T u_max_)
{
    u_max = u_max_;
}

template <typename T>
T control_system::pid::PID_LPF_1<T>::get_dt()
{
    return dt;
}

template <typename T>
T control_system::pid::PID_LPF_1<T>::get_Kp()
{
    return Kp;
}

template <typename T>
T control_system::pid::PID_LPF_1<T>::get_Ki()
{
    return Ki;
}

template <typename T>
T control_system::pid::PID_LPF_1<T>::get_Kd()
{
    return Kd;
}

template <typename T>
T control_system::pid::PID_LPF_1<T>::get_fc()
{
    return 1.0 / (6.28318530717958647692528676655900576 * tau);
}

template <typename T>
T control_system::pid::PID_LPF_1<T>::get_u_k_1()
{
    return u_k_1;
}

template <typename T>
T control_system::pid::PID_LPF_1<T>::get_e_k_1()
{
    return e_k_1;
}

template <typename T>
T control_system::pid::PID_LPF_1<T>::get_e_k_2()
{
    return e_k_2;
}

template <typename T>
T control_system::pid::PID_LPF_1<T>::get_u_max()
{
    return u_max;
}
