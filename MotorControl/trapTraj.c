#include "MyProject.h"


/****************************************************************************/
TRAPTRAJ_Config_t  trapTraj_config;
TRAPTRAJ_t  trap_traj_;

SCURVETRAJ_Config_t  sCurveTraj_config;
SCURVETRAJ_t  s_curve_traj_;
bool s_curve_trajectory_done_ = false;  // �켣��ɱ�־

/****************************************************************************/
void trapTraj_config_default(void)
{
	trapTraj_config.vel_limit = TRAPTraj_vel_limit;      //2.0f; // [turn/s]
	trapTraj_config.accel_limit = TRAPTraj_accel_limit;  //0.5f; // [turn/s^2]
	trapTraj_config.decel_limit = TRAPTraj_decel_limit;  //0.5f; // [turn/s^2]
}

/****************************************************************************/
// A sign function where input 0 has positive sign (not 0)
float sign_hard(float val)
{
	return signbit(val) ? -1.0f : 1.0f;
}
/****************************************************************************/
// Symbol                     Description
// Ta, Tv and Td              Duration of the stages of the AL profile
// Xi and Vi                  Adapted initial conditions for the AL profile
// Xf                         Position set-point
// s                          Direction (sign) of the trajectory
// Vmax, Amax, Dmax and jmax  Kinematic bounds
// Ar, Dr and Vr              Reached values of acceleration and velocity

// uint8_t planTrapezoidal(float Xf, float Xi, float Vi,float Vmax, float Amax, float Dmax)
// {
// 	float dX = Xf - Xi;  // Distance to travel
// 	float stop_dist = (Vi * Vi) / (2.0f * Dmax); // Minimum stopping distance
// 	float dXstop = copysignf(stop_dist, Vi); // Minimum stopping displacement
// 	float s = sign_hard(dX - dXstop); // Sign of coast velocity (if any)
// 	TRAPTRAJ_t  *p;
// 	p = &trap_traj_;
	
// 	p->Ar_ = s * Amax;  // Maximum Acceleration (signed)
// 	p->Dr_ = -s * Dmax; // Maximum Deceleration (signed)
// 	p->Vr_ = s * Vmax;  // Maximum Velocity (signed)
	
// 	// If we start with a speed faster than cruising, then we need to decel instead of accel
// 	// aka "double deceleration move" in the paper
// 	if ((s * Vi) > (s * p->Vr_))p->Ar_ = -s * Amax;
	
// 	// Time to accel/decel to/from Vr (cruise speed)
// 	p->Ta_ = (p->Vr_ - Vi) / p->Ar_;
// 	p->Td_ = -p->Vr_ / p->Dr_;
	
// 	// Integral of velocity ramps over the full accel and decel times to get
// 	// minimum displacement required to reach cuising speed
// 	float dXmin = 0.5f * p->Ta_ * (p->Vr_ + Vi) + 0.5f * p->Td_ * p->Vr_;
	
// 	// Are we displacing enough to reach cruising speed?
// 	if (s*dX < s*dXmin)
// 	{
// 		// Short move (triangle profile)
// 		p->Vr_ = s * sqrtf(max((p->Dr_ * SQ(Vi) + 2 * p->Ar_ * p->Dr_ * dX) / (p->Dr_ - p->Ar_), 0.0f));
// 		p->Ta_ = max(0.0f, (p->Vr_ - Vi) / p->Ar_);
// 		p->Td_ = max(0.0f, -p->Vr_ / p->Dr_);
// 		p->Tv_ = 0.0f;
// 	}
// 	else
// 	{
// 		// Long move (trapezoidal profile)
// 		p->Tv_ = (dX - dXmin) / p->Vr_;
// 	}

// 	// Fill in the rest of the values used at evaluation-time
// 	p->Tf_ = p->Ta_ + p->Tv_ + p->Td_;
// 	p->Xi_ = Xi;
// 	p->Xf_ = Xf;
// 	p->Vi_ = Vi;
// 	p->yAccel_ = Xi + Vi * p->Ta_ + 0.5f * p->Ar_ * SQ(p->Ta_); // pos at end of accel phase
	
// 	return 1;
// }

uint8_t planTrapezoidal(float Xf, float Xi, float Vi, float Vmax, float Amax, float Dmax)
{
    // 1. ������ȫ��飨����ԭ���߼��������Ͻ���
    if (Amax <= 0.0f || Dmax <= 0.0f || Vmax <= 0.0f) {
        // ������Ч������Ĭ��ֵ��������������
        TRAPTRAJ_t *p = &trap_traj_;
        p->Ta_ = p->Tv_ = p->Td_ = p->Tf_ = 0.0f;
        p->Vr_ = 0.0f;
        p->Ar_ = p->Dr_ = 0.0f;
        p->Xi_ = Xi;
        p->Xf_ = Xf;
        p->Vi_ = Vi;
        p->yAccel_ = Xi;
        return 0;
    }
    
    float dX = Xf - Xi;  // Distance to travel
    
    // 2. ��λ�ƻ�Сλ�ƴ���
    if (fabsf(dX) < 1e-6f) {
        // λ�Ƽ�С��ֱ��ֹͣ
        TRAPTRAJ_t *p = &trap_traj_;
        p->Ta_ = p->Tv_ = p->Td_ = p->Tf_ = 0.0f;
        p->Vr_ = 0.0f;
        p->Ar_ = p->Dr_ = 0.0f;
        p->Xi_ = Xi;
        p->Xf_ = Xf;
        p->Vi_ = Vi;
        p->yAccel_ = Xi;
        return 1;
    }
    
    // 3. ����ȷ���Ż�
    float s = (dX > 0.0f) ? 1.0f : -1.0f;  // λ�Ʒ���
    
    // 4. �ؼ�������ֹͣ�������Ӧ��ʹ�õ�ǰ�ٶȷ��򣬶�����λ�Ʒ���
    // ԭ�������⣺��Vi������λ�Ʒ����෴ʱ��stop_dist������Ż���
    float stop_dist = (Vi * Vi) / (2.0f * Dmax);
    float dXstop = (Vi >= 0) ? stop_dist : -stop_dist;  // ��ȷ����
    
    // 5. �ж��Ƿ���Ҫ�ȼ��ٵ�0
    // �����ʼ�ٶȷ�����Ŀ��λ�Ʒ����෴�������ȼ���
    if (Vi * s < 0) {
        // ��ʼ�ٶȷ��������Ҫ�ȼ��ٵ�0
        // ������ٵ�0�������
        float t_decel = fabsf(Vi) / Dmax;
        float decel_dist = Vi * t_decel + 0.5f * (-s * Dmax) * t_decel * t_decel;
        
        // ������ʼλ�ã�����شӼ��ٽ����㿪ʼ��
        Xi = Xi + decel_dist;
        dX = Xf - Xi;
        Vi = 0.0f;  // ��0�ٶȿ�ʼ
        
        // ���¼��㷽��
        s = (dX > 0.0f) ? 1.0f : -1.0f;
    }
    
    TRAPTRAJ_t *p;
    p = &trap_traj_;
    
    // 6. ���������ȷ���˶�����
    // �ؼ�������ȷ�����ٶȺͼ��ٶȵķ�����ȷ
    float signed_Vmax = s * Vmax;
    float signed_Amax = s * Amax;
    float signed_Dmax = -s * Dmax;  // ���ٶȷ�����λ�Ʒ����෴
    
    p->Ar_ = signed_Amax;
    p->Dr_ = signed_Dmax;
    p->Vr_ = signed_Vmax;
    
    // 7. ����ԭ������߼�����
    // ԭ���룺if ((s * Vi) > (s * p->Vr_)) ��Ϊ��
    if ((s * Vi) > (s * signed_Vmax)) {
        // ��ʼ�ٶȳ�������ٶȣ���Ҫ�ȼ���
        p->Ar_ = -signed_Amax;  // ʵ���ϱ�ɼ��ٶ�
    }
    
    // 8. ʱ������Ż�
    // ��ֹ����͸�ʱ��
    p->Ta_ = 0.0f;
    p->Td_ = 0.0f;
    
    if (fabsf(p->Ar_) > 1e-6f) {
        p->Ta_ = (p->Vr_ - Vi) / p->Ar_;
        if (p->Ta_ < 0.0f) p->Ta_ = 0.0f;
    }
    
    if (fabsf(p->Dr_) > 1e-6f) {
        p->Td_ = -p->Vr_ / p->Dr_;
        if (p->Td_ < 0.0f) p->Td_ = 0.0f;
    }
    
    // 9. �ؼ�������dXmin�����Ż�
    // ԭ�����dXmin��ʽ��p->Ta_��p->Td_Ϊ��ʱ������
    float dXmin = 0.0f;
    if (p->Ta_ > 0.0f) {
        dXmin += 0.5f * p->Ta_ * (p->Vr_ + Vi);
    }
    if (p->Td_ > 0.0f) {
        dXmin += 0.5f * p->Td_ * p->Vr_;
    }
    
    // 10. �жϹ켣���ͣ������λ����Σ��Ż�
    // ʹ�þ���ֵ�Ƚϣ������������
    if (fabsf(dX) <= fabsf(dXmin) + 1e-6f) {
        // �����ι켣��λ�Ʋ����Դﵽ����ٶȣ�
        
        // �ؼ�������ʹ�ø��ȶ��������ι켣����
        // ԭ��ʽ��������㵼������
        float numerator = p->Dr_ * Vi * Vi + 2.0f * p->Ar_ * p->Dr_ * dX;
        float denominator = p->Dr_ - p->Ar_;
        
        // ��ֹ����͸�ֵ
        float sqrt_input = 0.0f;
        if (fabsf(denominator) > 1e-6f) {
            sqrt_input = numerator / denominator;
        }
        
        // ȷ��sqrt_input�Ǹ�
        if (sqrt_input < 0.0f) sqrt_input = 0.0f;
        
        p->Vr_ = s * sqrtf(sqrt_input);
        
        // ȷ����������ٶȲ�����Vmax
        if (fabsf(p->Vr_) > Vmax) {
            p->Vr_ = s * Vmax;
        }
        
        // ���¼���ʱ��
        if (fabsf(p->Ar_) > 1e-6f) {
            p->Ta_ = (p->Vr_ - Vi) / p->Ar_;
            if (p->Ta_ < 0.0f) p->Ta_ = 0.0f;
        }
        
        if (fabsf(p->Dr_) > 1e-6f) {
            p->Td_ = -p->Vr_ / p->Dr_;
            if (p->Td_ < 0.0f) p->Td_ = 0.0f;
        }
        
        p->Tv_ = 0.0f;
    } else {
        // ���ι켣�������ٶΣ�
        p->Tv_ = (dX - dXmin) / p->Vr_;
        if (p->Tv_ < 0.0f) p->Tv_ = 0.0f;
    }
    
    // 11. ����ʱ��������֤
    p->Tf_ = p->Ta_ + p->Tv_ + p->Td_;
    
    // 12. �ؼ���������֤�켣λ��һ����
    // ����ʵ�ʲ�����λ��
    float Xa = Vi * p->Ta_ + 0.5f * p->Ar_ * p->Ta_ * p->Ta_;
    float Xv = p->Vr_ * p->Tv_;
    float Xd = p->Vr_ * p->Td_ + 0.5f * p->Dr_ * p->Td_ * p->Td_;
    float total_displacement = Xa + Xv + Xd;
    
    // �������λ����ʵ��λ���нϴ����������ٶ�ʱ��
    float error = dX - total_displacement;
    if (fabsf(error) > 1e-3f * fabsf(dX) && fabsf(p->Vr_) > 1e-6f) {
        // ΢�����ٶ��Բ����������
        p->Tv_ += error / p->Vr_;
        p->Tf_ = p->Ta_ + p->Tv_ + p->Td_;
    }
    
    // 13. ���ʣ�����
    p->Xi_ = Xi;
    p->Xf_ = Xf;
    p->Vi_ = Vi;  // ע�⣺���֮ǰ��������ʼ�ٶȣ������ǵ������ֵ
    
    // ����yAccel_���㣬ȷ��������ʱ�䵼�´���
    if (p->Ta_ >= 0.0f) {
        p->yAccel_ = Xi + Vi * p->Ta_ + 0.5f * p->Ar_ * p->Ta_ * p->Ta_;
    } else {
        p->yAccel_ = Xi;  // �޼��ٶ�
    }
    
    // 14. ������֤��ȷ������ʱ��Ǹ�
    p->Ta_ = (p->Ta_ > 0.0f) ? p->Ta_ : 0.0f;
    p->Tv_ = (p->Tv_ > 0.0f) ? p->Tv_ : 0.0f;
    p->Td_ = (p->Td_ > 0.0f) ? p->Td_ : 0.0f;
    
    return 1;
}

/****************************************************************************/
// Step_t trap_traj_eval(float t)
// {
// 	Step_t trajStep;
	
//     if (t < 0.0f) {  // Initial Condition
//         trajStep.Y   = trap_traj_.Xi_;
//         trajStep.Yd  = trap_traj_.Vi_;
//         trajStep.Ydd = 0.0f;
//     } else if (t < trap_traj_.Ta_) {  // Accelerating
//         trajStep.Y   = trap_traj_.Xi_ + trap_traj_.Vi_ * t + 0.5f * trap_traj_.Ar_ * SQ(t);
//         trajStep.Yd  = trap_traj_.Vi_ + trap_traj_.Ar_*t;
//         trajStep.Ydd = trap_traj_.Ar_;
//     } else if (t < trap_traj_.Ta_ + trap_traj_.Tv_) {  // Coasting
//         trajStep.Y   = trap_traj_.yAccel_ + trap_traj_.Vr_*(t - trap_traj_.Ta_);
//         trajStep.Yd  = trap_traj_.Vr_;
//         trajStep.Ydd = 0.0f;
//     } else if (t < trap_traj_.Tf_) {  // Deceleration
//         float td     = t - trap_traj_.Tf_;
//         trajStep.Y   = trap_traj_.Xf_ + 0.5f * trap_traj_.Dr_ * SQ(td);
//         trajStep.Yd  = trap_traj_.Dr_*td;
//         trajStep.Ydd = trap_traj_.Dr_;
//     } else if (t >= trap_traj_.Tf_) {  // Final Condition
//         trajStep.Y   = trap_traj_.Xf_;
//         trajStep.Yd  = 0.0f;
//         trajStep.Ydd = 0.0f;
//     } else {
//         // TODO: report error here
//     }

//     return trajStep;
// }

Step_t trap_traj_eval(float t)
{
    Step_t trajStep;
    
    // 1. ʹ�þֲ����������η��ʽṹ��
    float Ta = trap_traj_.Ta_;
    float Tv = trap_traj_.Tv_;
    float Td = trap_traj_.Td_;
    float Tf = trap_traj_.Tf_;
    float Xi = trap_traj_.Xi_;
    float Xf = trap_traj_.Xf_;
    float Vi = trap_traj_.Vi_;
    float Vr = trap_traj_.Vr_;
    float Ar = trap_traj_.Ar_;
    float Dr = trap_traj_.Dr_;
    float yAccel = trap_traj_.yAccel_;
    
    // 2. ����epsilon�������㾫������
    const float EPS = 1e-6f;
    
    // 3. ��ʼ�����ж��Ż�
    if (t < 0.0f) {
        trajStep.Y = Xi;
        trajStep.Yd = Vi;
        trajStep.Ydd = 0.0f;
        return trajStep;
    }
    
    // 4. ʱ��߽��ж��Ż������Ǹ��㾫��
    // ���ٶ��жϣ�t < Ta�������Ta�ӽ�0��ֱ������
    if (Ta > EPS && t < Ta - EPS) {
        // Accelerating
        trajStep.Y = Xi + Vi * t + 0.5f * Ar * t * t;
        trajStep.Yd = Vi + Ar * t;
        trajStep.Ydd = Ar;
    }
    // ���ٶ��жϣ�t >= Ta �� t < (Ta + Tv)
    else if (Tv > EPS && t >= Ta - EPS && t < Ta + Tv - EPS) {
        // Coasting
        float t_coast = t - Ta;
        // �ؼ�������ʹ�ø�׼ȷ�����ٶ�������
        // ԭ����ֱ��ʹ��yAccel������Ҫ���Ǽ��ٶθպ�Ϊ0�����
        float coast_start = yAccel;
        // ������ٶ�ʱ��Ϊ0�����ٶ����Ӧ���ǳ�ʼλ��
        if (Ta <= EPS) {
            coast_start = Xi;
        }
        
        trajStep.Y = coast_start + Vr * t_coast;
        trajStep.Yd = Vr;
        trajStep.Ydd = 0.0f;
    }
    // ���ٶ��жϣ�t >= (Ta + Tv) �� t < Tf
    else if (Td > EPS && t >= Ta + Tv - EPS && t < Tf - EPS) {
        // Deceleration
        // �ؼ�������ԭ����ļ��ٶμ���������
        // ��ȷ�ļ��ٶι�ʽ���Ӽ�����㿪ʼ����
        
        // ������ٶ����ʱ���λ��
        float t_decel_start = Ta + Tv;
        float decel_start_pos = 0.0f;
        
        if (Tv > EPS) {
            // �����ٶε����
            decel_start_pos = yAccel + Vr * Tv;
        } else {
            // �����ٶε�����������ι켣��
            if (Ta > EPS) {
                decel_start_pos = yAccel;  // ���ٶν���λ�þ��Ǽ��ٶ����
            } else {
                // ֻ�м��ٶε����
                decel_start_pos = Xi;
            }
        }
        
        // ���ٶ��ڵ�ʱ��ƫ��
        float t_decel = t - t_decel_start;
        
        // ���ٶ��˶�ѧ��ʽ
        trajStep.Y = decel_start_pos + Vr * t_decel + 0.5f * Dr * t_decel * t_decel;
        trajStep.Yd = Vr + Dr * t_decel;
        trajStep.Ydd = Dr;
        
        // ��֤���ڼ��ٶν���ʱλ��Ӧ�ýӽ�Xf
        if (t_decel > Td - EPS && t_decel < Td + EPS) {
            // �ӽ����ٶν�����ǿ������ΪĿ��λ�ú��ٶ�
            float expected_final_pos = Xf;
            float pos_error = fabsf(trajStep.Y - expected_final_pos);
            
            if (pos_error > 1e-3f) {
                // λ�����ϴ󣬽�������
                trajStep.Y = expected_final_pos;
                trajStep.Yd = 0.0f;  // �����ٶ�Ϊ0
                trajStep.Ydd = 0.0f;
            }
        }
    }
    // ���������жϣ�t >= Tf
    else if (t >= Tf - EPS) {
        // Final Condition
        trajStep.Y = Xf;
        trajStep.Yd = 0.0f;
        trajStep.Ydd = 0.0f;
    }
    // �����߽������������
    else {
        // �������������ĳһʱ���Ϊ0�����
        
        // ���1��û�м��ٶΣ�Ta �� 0��
        if (Ta <= EPS && Tv > EPS && t < Tv - EPS) {
            // ֱ�ӽ������ٶ�
            trajStep.Y = Xi + Vr * t;
            trajStep.Yd = Vr;
            trajStep.Ydd = 0.0f;
        }
        // ���2��û�����ٶΣ�Tv �� 0��
        else if (Ta > EPS && Tv <= EPS && t >= Ta - EPS && t < Tf - EPS) {
            // ���ٺ�ֱ�Ӽ���
            float t_decel = t - Ta;
            trajStep.Y = yAccel + Vr * t_decel + 0.5f * Dr * t_decel * t_decel;
            trajStep.Yd = Vr + Dr * t_decel;
            trajStep.Ydd = Dr;
        }
        // ���3��ֻ�м��ٶ�
        else if (Ta <= EPS && Tv <= EPS && Td > EPS && t < Td - EPS) {
            // ֻ�м��ٶ�
            trajStep.Y = Xi + Vi * t + 0.5f * Dr * t * t;
            trajStep.Yd = Vi + Dr * t;
            trajStep.Ydd = Dr;
        }
        // ���4��˲����ɣ����жζ�Ϊ0��
        else {
            trajStep.Y = Xf;
            trajStep.Yd = 0.0f;
            trajStep.Ydd = 0.0f;
        }
    }
    
    // 5. ������֤������
    // ȷ��λ�ò��ᳬ�������յ�
    float min_pos = (Xi < Xf) ? Xi : Xf;
    float max_pos = (Xi > Xf) ? Xi : Xf;
    
    if (trajStep.Y < min_pos - 1e-3f || trajStep.Y > max_pos + 1e-3f) {
        // λ���쳣����������
        if (trajStep.Y < min_pos) trajStep.Y = min_pos;
        if (trajStep.Y > max_pos) trajStep.Y = max_pos;
    }
    
    // 6. �ٶȷ�����֤
    // �����Xi��Xf�������˶����ٶ�ӦΪ�Ǹ��������˶���Ϊ����
    float dir = (Xf > Xi) ? 1.0f : -1.0f;
    if (dir > 0 && trajStep.Yd < -1e-3f) {
        // �����˶��в�Ӧ���ָ��ٶȣ������г�����
        if (t < Tf - EPS) {
            trajStep.Yd = fmaxf(trajStep.Yd, 0.0f);
        }
    } else if (dir < 0 && trajStep.Yd > 1e-3f) {
        // �����˶��в�Ӧ�������ٶ�
        if (t < Tf - EPS) {
            trajStep.Yd = fminf(trajStep.Yd, 0.0f);
        }
    }
    
    // 7. �ڽӽ��յ�ʱǿ�ƹ����ٶȺͼ��ٶ�
    if (t >= Tf - EPS) {
        trajStep.Y = Xf;
        trajStep.Yd = 0.0f;
        trajStep.Ydd = 0.0f;
    }
    
    return trajStep;
}

/****************************************************************************/

void sCurveTraj_config_default(void)
{
    // ����Ĭ�ϲ������ɵ�
    sCurveTraj_config.vel_limit = 20.0f;     // [turn/s]
    sCurveTraj_config.accel_limit = 2.0f;   // [turn/s^2]
    sCurveTraj_config.decel_limit = 2.0f;   // [turn/s^2]
    sCurveTraj_config.jerk_limit = 1.0f;    // [turn/s^3] - S�͹켣���в���
}

/****************************************************************************/
// ��������������ƽ��������
static inline float SQ_(float x) { return x * x; }
static inline float CB(float x) { return x * x * x; }


// S�͹켣�滮���� - ���������ι켣��ͬ�����ӿ�
// ����jerk���������ƼӼ��ٶ�
uint8_t planSCurve(float Xf, float Xi, float Vi, float Vmax, float Amax, float Dmax, float Jmax)
{
    float dX = Xf - Xi;  // ��λ��
    float s = sign_hard(dX); // �˶�����
    
    SCURVETRAJ_t *p = &s_curve_traj_;
    
    // �������
    p->Xi_ = Xi;
    p->Xf_ = Xf;
    p->Vi_ = Vi;
    
    // �з��Ų���
    p->Vmax_ = s * Vmax;
    p->Amax_ = s * Amax;
    p->Dmax_ = -s * Dmax;  // ���ٶ�Ϊ��
    p->Jmax_ = s * Jmax;
    
    // ����Ƿ���Ҫֹͣ�������ι켣��ͬ��ֹͣ������㣩
    float stop_dist = (Vi * Vi) / (2.0f * Dmax);
    float dXstop = copysignf(stop_dist, Vi);
    
    // �����ʼ�ٶȷ������˶������෴����Ҫ�ȼ��ٵ�0
    if ((s * Vi) < 0) {
        // �ȼ��ٵ�0��Ȼ���ٿ�ʼS�͹켣
        // �򻯴����������0��ʼ
        p->Vi_ = 0.0f;
        dX = Xf - (Xi + dXstop);
        s = sign_hard(dX);
    }
    
    // ����Ӽ��ٶȽ׶�ʱ�䣨�ﵽ�����ٶ�����ʱ�䣩
    p->Tj1_ = Amax / Jmax;
    p->Tj2_ = Dmax / Jmax;
    
    // �ж��Ƿ��ܴﵽ����ٶ�
    float dXmin_accel = 0.0f;
    float dXmin_decel = 0.0f;
    
    // ���ٶ���Сλ�Ƽ��㣨��Vi���ٵ�Vmax��
    if (fabsf(p->Vmax_ - Vi) < Amax * p->Tj1_) {
        // �޷��ﵽ�����ٶȣ������μ���
        float Tj_adj = sqrtf(fabsf(p->Vmax_ - Vi) / Jmax);
        dXmin_accel = Vi * Tj_adj + (1.0f/6.0f) * Jmax * Tj_adj * Tj_adj * Tj_adj;
    } else {
        // ���Դﵽ�����ٶȣ����μ���
        dXmin_accel = Vi * p->Tj1_ + (1.0f/6.0f) * Jmax * p->Tj1_ * p->Tj1_ * p->Tj1_;
        dXmin_accel += (Vi + 0.5f * Jmax * p->Tj1_ * p->Tj1_) * 
                      (fabsf(p->Vmax_ - Vi)/Amax - p->Tj1_);
        dXmin_accel += 0.5f * Amax * (fabsf(p->Vmax_ - Vi)/Amax - p->Tj1_) * 
                      (fabsf(p->Vmax_ - Vi)/Amax - p->Tj1_);
        dXmin_accel += (p->Vmax_ - 0.5f * Jmax * p->Tj1_ * p->Tj1_) * p->Tj1_;
        dXmin_accel -= (1.0f/6.0f) * Jmax * p->Tj1_ * p->Tj1_ * p->Tj1_;
    }
    
    // ���ٶ���Сλ�Ƽ��㣨��Vmax���ٵ�0��
    if (fabsf(p->Vmax_) < Dmax * p->Tj2_) {
        // �޷��ﵽ�����ٶȣ������μ���
        float Tj_adj = sqrtf(fabsf(p->Vmax_) / Jmax);
        dXmin_decel = (1.0f/6.0f) * Jmax * Tj_adj * Tj_adj * Tj_adj;
    } else {
        // ���Դﵽ�����ٶȣ����μ���
        dXmin_decel = (1.0f/6.0f) * Jmax * p->Tj2_ * p->Tj2_ * p->Tj2_;
        dXmin_decel += (0.5f * Jmax * p->Tj2_ * p->Tj2_) * 
                      (fabsf(p->Vmax_)/Dmax - p->Tj2_);
        dXmin_decel += 0.5f * Dmax * (fabsf(p->Vmax_)/Dmax - p->Tj2_) * 
                      (fabsf(p->Vmax_)/Dmax - p->Tj2_);
        dXmin_decel += (p->Vmax_ - 0.5f * Jmax * p->Tj2_ * p->Tj2_) * p->Tj2_;
        dXmin_decel -= (1.0f/6.0f) * Jmax * p->Tj2_ * p->Tj2_ * p->Tj2_;
    }
    
    float dXmin = fabsf(dXmin_accel) + fabsf(dXmin_decel);
    
    // �ж��Ƿ������ٶ�
    if (s * dX < s * dXmin) {
        // �̾��룺�����ٶΣ�������S���ߣ�
        p->has_coast_ = 0;
        
        // ��Ҫ����ʵ���ܴﵽ������ٶ�
        // �򻯼��㣺ʹ�ö��ַ�
        float V_low = fabsf(Vi);
        float V_high = Vmax;
        float V_mid = (V_low + V_high) / 2.0f;
        
        for (int i = 0; i < 10; i++) { // ����10��
            // ���㵱ǰ�ٶ��µ�λ��
            float dX_test = 0.0f;
            
            // ���ٶ�λ��
            if (fabsf(V_mid - Vi) < Amax * p->Tj1_) {
                float Tj_acc = sqrtf(fabsf(V_mid - Vi) / Jmax);
                dX_test += Vi * Tj_acc + (1.0f/6.0f) * Jmax * Tj_acc * Tj_acc * Tj_acc;
            } else {
                dX_test += Vi * p->Tj1_ + (1.0f/6.0f) * Jmax * p->Tj1_ * p->Tj1_ * p->Tj1_;
                dX_test += (Vi + 0.5f * Jmax * p->Tj1_ * p->Tj1_) * 
                          (fabsf(V_mid - Vi)/Amax - p->Tj1_);
                dX_test += 0.5f * Amax * (fabsf(V_mid - Vi)/Amax - p->Tj1_) * 
                          (fabsf(V_mid - Vi)/Amax - p->Tj1_);
                dX_test += (V_mid - 0.5f * Jmax * p->Tj1_ * p->Tj1_) * p->Tj1_;
                dX_test -= (1.0f/6.0f) * Jmax * p->Tj1_ * p->Tj1_ * p->Tj1_;
            }
            
            // ���ٶ�λ��
            if (fabsf(V_mid) < Dmax * p->Tj2_) {
                float Tj_dec = sqrtf(fabsf(V_mid) / Jmax);
                dX_test += (1.0f/6.0f) * Jmax * Tj_dec * Tj_dec * Tj_dec;
            } else {
                dX_test += (1.0f/6.0f) * Jmax * p->Tj2_ * p->Tj2_ * p->Tj2_;
                dX_test += (0.5f * Jmax * p->Tj2_ * p->Tj2_) * 
                          (fabsf(V_mid)/Dmax - p->Tj2_);
                dX_test += 0.5f * Dmax * (fabsf(V_mid)/Dmax - p->Tj2_) * 
                          (fabsf(V_mid)/Dmax - p->Tj2_);
                dX_test += (V_mid - 0.5f * Jmax * p->Tj2_ * p->Tj2_) * p->Tj2_;
                dX_test -= (1.0f/6.0f) * Jmax * p->Tj2_ * p->Tj2_ * p->Tj2_;
            }
            
            if (fabsf(dX_test) < fabsf(dX)) {
                V_low = V_mid;
            } else {
                V_high = V_mid;
            }
            V_mid = (V_low + V_high) / 2.0f;
        }
        
        p->Vr_ = s * V_mid;
        
        // ���¼������ʱ��
        float Vr_abs = fabsf(p->Vr_);
        
        // ���ٶ�ʱ��
        if (Vr_abs - fabsf(Vi) < Amax * p->Tj1_) {
            float Tj_acc = sqrtf((Vr_abs - fabsf(Vi)) / Jmax);
            p->Tj1_ = Tj_acc;
            p->Ta_ = 2.0f * Tj_acc;
        } else {
            p->Ta_ = 2.0f * p->Tj1_ + (Vr_abs - fabsf(Vi) - Amax * p->Tj1_) / Amax;
        }
        
        // ���ٶ�ʱ��
        if (Vr_abs < Dmax * p->Tj2_) {
            float Tj_dec = sqrtf(Vr_abs / Jmax);
            p->Tj2_ = Tj_dec;
            p->Td_ = 2.0f * Tj_dec;
        } else {
            p->Td_ = 2.0f * p->Tj2_ + (Vr_abs - Dmax * p->Tj2_) / Dmax;
        }
        
        p->Tv_ = 0.0f;
    } else {
        // �����룺�����ٶΣ�����S���ߣ�
        p->has_coast_ = 1;
        p->Vr_ = p->Vmax_;
        
        // ���ٶ�ʱ��
        p->Ta_ = 2.0f * p->Tj1_ + (fabsf(p->Vr_ - Vi) - Amax * p->Tj1_) / Amax;
        
        // ���ٶ�ʱ��
        p->Td_ = 2.0f * p->Tj2_ + (fabsf(p->Vr_) - Dmax * p->Tj2_) / Dmax;
        
        // �������ٶ�ʱ��
        // ���ٶ�λ��
        float Sa = 0.0f;
        if (fabsf(p->Vr_ - Vi) < Amax * p->Tj1_) {
            float Tj_acc = sqrtf(fabsf(p->Vr_ - Vi) / Jmax);
            Sa = Vi * Tj_acc + (1.0f/6.0f) * Jmax * Tj_acc * Tj_acc * Tj_acc;
        } else {
            Sa = Vi * p->Tj1_ + (1.0f/6.0f) * Jmax * p->Tj1_ * p->Tj1_ * p->Tj1_;
            Sa += (Vi + 0.5f * Jmax * p->Tj1_ * p->Tj1_) * 
                  (fabsf(p->Vr_ - Vi)/Amax - p->Tj1_);
            Sa += 0.5f * Amax * (fabsf(p->Vr_ - Vi)/Amax - p->Tj1_) * 
                  (fabsf(p->Vr_ - Vi)/Amax - p->Tj1_);
            Sa += (p->Vr_ - 0.5f * Jmax * p->Tj1_ * p->Tj1_) * p->Tj1_;
            Sa -= (1.0f/6.0f) * Jmax * p->Tj1_ * p->Tj1_ * p->Tj1_;
        }
        
        // ���ٶ�λ��
        float Sd = 0.0f;
        if (fabsf(p->Vr_) < Dmax * p->Tj2_) {
            float Tj_dec = sqrtf(fabsf(p->Vr_) / Jmax);
            Sd = (1.0f/6.0f) * Jmax * Tj_dec * Tj_dec * Tj_dec;
        } else {
            Sd = (1.0f/6.0f) * Jmax * p->Tj2_ * p->Tj2_ * p->Tj2_;
            Sd += (0.5f * Jmax * p->Tj2_ * p->Tj2_) * 
                  (fabsf(p->Vr_)/Dmax - p->Tj2_);
            Sd += 0.5f * Dmax * (fabsf(p->Vr_)/Dmax - p->Tj2_) * 
                  (fabsf(p->Vr_)/Dmax - p->Tj2_);
            Sd += (p->Vr_ - 0.5f * Jmax * p->Tj2_ * p->Tj2_) * p->Tj2_;
            Sd -= (1.0f/6.0f) * Jmax * p->Tj2_ * p->Tj2_ * p->Tj2_;
        }
        
        // ���ٶ�ʱ��
        p->Tv_ = (dX - Sa - Sd) / p->Vr_;
    }
    
    // ������ʱ��
    p->T_ = p->Ta_ + p->Tv_ + p->Td_;
    
    // ������ٶν���λ�ã����ڿ��ټ��㣩
    p->yAccel_ = Xi;
    if (fabsf(p->Vr_ - Vi) < Amax * p->Tj1_) {
        float Tj_acc = sqrtf(fabsf(p->Vr_ - Vi) / Jmax);
        p->yAccel_ += Vi * Tj_acc + (1.0f/6.0f) * Jmax * Tj_acc * Tj_acc * Tj_acc;
    } else {
        p->yAccel_ += Vi * p->Tj1_ + (1.0f/6.0f) * Jmax * p->Tj1_ * p->Tj1_ * p->Tj1_;
        p->yAccel_ += (Vi + 0.5f * Jmax * p->Tj1_ * p->Tj1_) * 
                     (fabsf(p->Vr_ - Vi)/Amax - p->Tj1_);
        p->yAccel_ += 0.5f * Amax * (fabsf(p->Vr_ - Vi)/Amax - p->Tj1_) * 
                     (fabsf(p->Vr_ - Vi)/Amax - p->Tj1_);
        p->yAccel_ += (p->Vr_ - 0.5f * Jmax * p->Tj1_ * p->Tj1_) * p->Tj1_;
        p->yAccel_ -= (1.0f/6.0f) * Jmax * p->Tj1_ * p->Tj1_ * p->Tj1_;
    }
    
    // ������ٶο�ʼλ��
    p->yDecel_ = p->yAccel_ + p->Vr_ * p->Tv_;
    
    return 1;
}

// �򻯰�S���߹滮�������ι켣�ӿ���ȫ���ݣ�
uint8_t planSCurve_simple(float Xf, float Xi, float Vi)
{
    // ʹ��Ĭ������
    return planSCurve(Xf, Xi, Vi,
                     sCurveTraj_config.vel_limit,
                     sCurveTraj_config.accel_limit,
                     sCurveTraj_config.decel_limit,
                     sCurveTraj_config.jerk_limit);
}


// S�͹켣�������� - ���������ι켣��ͬ�ķ�������
Step_t s_curve_traj_eval(float t)
{
    Step_t trajStep;
    SCURVETRAJ_t *p = &s_curve_traj_;
    
    float dt = 0.0f;
    
    if (t < 0.0f) {  // ��ʼ����
        trajStep.Y   = p->Xi_;
        trajStep.Yd  = p->Vi_;
        trajStep.Ydd = 0.0f;
    } else if (t < p->Tj1_) {  // ��һ�׶Σ��Ӽ��ٶ����� (0 �� Jmax)
        trajStep.Y   = p->Xi_ + p->Vi_ * t + (1.0f/6.0f) * p->Jmax_ * t * t * t;
        trajStep.Yd  = p->Vi_ + 0.5f * p->Jmax_ * t * t;
        trajStep.Ydd = p->Jmax_ * t;
    } else if (t < p->Ta_ - p->Tj1_) {  // �ڶ��׶Σ�����ٶȶ�
        dt = t - p->Tj1_;
        trajStep.Ydd = p->Amax_;
        trajStep.Yd  = p->Vi_ + 0.5f * p->Jmax_ * p->Tj1_ * p->Tj1_ + p->Amax_ * dt;
        trajStep.Y   = p->Xi_ + p->Vi_ * p->Tj1_ + (1.0f/6.0f) * p->Jmax_ * p->Tj1_ * p->Tj1_ * p->Tj1_
                      + (p->Vi_ + 0.5f * p->Jmax_ * p->Tj1_ * p->Tj1_) * dt
                      + 0.5f * p->Amax_ * dt * dt;
    } else if (t < p->Ta_) {  // �����׶Σ��Ӽ��ٶ��½� (Jmax �� 0)
        dt = t - (p->Ta_ - p->Tj1_);
        trajStep.Ydd = p->Amax_ - p->Jmax_ * dt;
        trajStep.Yd  = p->Vr_ - 0.5f * p->Jmax_ * (p->Tj1_ - dt) * (p->Tj1_ - dt);
        trajStep.Y   = p->yAccel_ + p->Vr_ * dt - 0.5f * p->Amax_ * dt * dt
                      + (1.0f/6.0f) * p->Jmax_ * dt * dt * dt;
    } else if (t < p->Ta_ + p->Tv_) {  // ���Ľ׶Σ����ٶ�
        dt = t - p->Ta_;
        trajStep.Y   = p->yAccel_ + p->Vr_ * dt;
        trajStep.Yd  = p->Vr_;
        trajStep.Ydd = 0.0f;
    } else if (t < p->Ta_ + p->Tv_ + p->Tj2_) {  // ����׶Σ������ٶ����� (0 �� -Jmax)
        dt = t - (p->Ta_ + p->Tv_);
        trajStep.Ydd = -p->Jmax_ * dt;
        trajStep.Yd  = p->Vr_ - 0.5f * p->Jmax_ * dt * dt;
        trajStep.Y   = p->yDecel_ + p->Vr_ * dt - (1.0f/6.0f) * p->Jmax_ * dt * dt * dt;
    } else if (t < p->T_ - p->Tj2_) {  // �����׶Σ�����ٶȶ�
        dt = t - (p->Ta_ + p->Tv_ + p->Tj2_);
        trajStep.Ydd = p->Dr_;
        trajStep.Yd  = p->Vr_ + p->Dr_ * dt;
        trajStep.Y   = p->yDecel_ + p->Vr_ * p->Tj2_ - (1.0f/6.0f) * p->Jmax_ * p->Tj2_ * p->Tj2_ * p->Tj2_
                      + (p->Vr_ - 0.5f * p->Jmax_ * p->Tj2_ * p->Tj2_) * dt
                      + 0.5f * p->Dr_ * dt * dt;
    } else if (t < p->T_) {  // ���߽׶Σ������ٶ��½� (-Jmax �� 0)
        dt = t - (p->T_ - p->Tj2_);
        trajStep.Ydd = p->Dr_ + p->Jmax_ * dt;
        trajStep.Yd  = p->Dr_ * dt + 0.5f * p->Jmax_ * dt * dt;
        trajStep.Y   = p->Xf_ + 0.5f * p->Dr_ * dt * dt + (1.0f/6.0f) * p->Jmax_ * dt * dt * dt;
    } else {  // ��������
        trajStep.Y   = p->Xf_;
        trajStep.Yd  = 0.0f;
        trajStep.Ydd = 0.0f;
    }
    
    return trajStep;
}
