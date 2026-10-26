#include "Quadrature_enc.h"

Quadrature_enc::Quadrature_enc()
{
}

void Quadrature_enc::config(uint64_t cprOutput, int pinA, int pinB, void (*isrHandler)(void))
{
    m_cprOutput = cprOutput;
    m_pinA = pinA;
    m_pinB = pinB;
    m_isrHandler = isrHandler;
}

void Quadrature_enc::init()
{
    attachInterrupt(digitalPinToInterrupt(m_pinA), m_isrHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(m_pinB), m_isrHandler, CHANGE);

    m_time = micros();
    m_timePrev = m_time;
}

void Quadrature_enc::isrFunction()
{
    a = digitalRead(m_pinA);
    b = digitalRead(m_pinB);

    if (a != a_prev)
    {
        // interrupt due to A
        if (a != b)
        {
            m_count++;
        }
        else
        {
            m_count--;
        }
    }
    else
    {
        // interrupt due to B
        if (a == b)
        {
            m_count++;
        }
        else
        {
            m_count--;
        }
    }

    a_prev = a;
    b_prev = b;
}

void Quadrature_enc::update()
{
    noInterrupts();
    m_count_temp = m_count;
    interrupts();

    m_angle = static_cast<double>(m_count_temp) / static_cast<double>(m_cprOutput) * 2.0 * M_PI;
    m_time = micros();

    uint64_t dt = m_time - m_timePrev;
    if (dt > 0)
    {
        m_angularSpeed = (m_angle - m_anglePrev) * 1e6 / (static_cast<double>(dt));
    }

    m_anglePrev = m_angle;
    m_timePrev = m_time;
}

double Quadrature_enc::getAngle()
{
    return m_angle;
}

double Quadrature_enc::getAngularSpeed()
{
    return m_angularSpeed;
}
