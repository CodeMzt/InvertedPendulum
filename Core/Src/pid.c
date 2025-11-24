/********************************************************************************

* @File pid.c

* @Author: Ma Ziteng

* @Version: 1.0

* @Date: 2025-11

* @Description: 闭环pid控制系统

********************************************************************************/

#include "../Inc/pid.h"

#include <math.h>

#include "Encoder.h"
#include "motor.h"

//速度环
/*        1.01          0.4        0     */
//位置环
/*        0.37          0        0.9   */



void PID_Init(PID_T* p) {

}

/**
* @brief    pid控制
* @param    p    结构体指针
* @return
*/
void PID_Update(PID_T* p) {
    float Err = p->Target - p->Current;

    //输入死区
    if (p->Flags&0x02 && fabsf(Err)<=p->InputDeadZone_Threshold) {
        p->Out = 0;
        return;
    }

    //微分先行
    float Err_Diff ;
    if (p->Flags&0x04)
        Err_Diff = -(p->Current - p->Last);
    else Err_Diff = Err - p->Err_Last;

    //低通滤波
    Err_Diff = p->Diff_Filter*Err_Diff + (1-p->Diff_Filter)*p->Diff_Last;

    //增量式
    if (p->Flags & 0x01) {
        p->Out += p->Ki*Err + p->Kp*(Err - p->Err_Last) + p->Kd*(Err_Diff - p->Diff_Last);
        p->Diff_Last = Err_Diff;
        return;
    }

    p->Last = p->Current;
    p->Err_Last = Err;
    p->Diff_Last = Err_Diff;

    float Err_Integral_Out=0;
    if (p->Ki!=0) {
        p->Err_Integral+=Err;
        //积分分离
        if (p->Flags & 0x08) {
            if (fabsf(Err)<=p->IntegralSeparation_Threshold)
                Err_Integral_Out = p->Err_Integral*p->Ki;
        }
        //变速积分
        else if (p->Flags & 0x10)
                Err_Integral_Out = 1.0/(p->VariableIntegral*Err+1) * p->Err_Integral * p->Ki;
        else Err_Integral_Out = p->Err_Integral*p->Ki;
    }
    Err_Integral_Out = Err_Integral_Out>p->Err_Integral_Out_Max?p->Err_Integral_Out_Max:(Err_Integral_Out<p->Err_Integral_Out_Min?p->Err_Integral_Out_Min:Err_Integral_Out);

    p->Out = Err*p->Kp + Err_Diff*p->Kd + Err_Integral_Out;
    p->Out = p->Out>p->Out_Max?p->Out_Max:(p->Out<p->Out_Min?p->Out_Min:p->Out);

    //输出偏移
    if (p->Flags & 0x20){
        if (p->Out > p->OutputOffset_Threshold)
            p->Out += p->OutputOffset;
        else if (p->Out < -p->OutputOffset_Threshold)
            p->Out -= p->OutputOffset;
    }
}





