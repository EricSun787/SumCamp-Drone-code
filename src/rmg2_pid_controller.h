#include <rmg2/rmg2_pid.h>
#define PID_RATE 20.0
float absd(float a)
{
    if(a>0)return a;
    else   return -a;
}
float rmg2_Xpid_control(rmg2::rmg2_pid& cpid)
{
      float pos_out_x = 0;
	
	static float pi_x = 0;
	static float pos_bias_x[2] = {0.0 , 0.0 };
	
	pos_bias_x[0] = pos_bias_x[1];
	pos_bias_x[1] = cpid.target - cpid.feedback;

	if((pos_bias_x[1] < 0.02) && (pos_bias_x[1] > -0.02) )
		pos_bias_x[1] = 0;
	
	pi_x += 0.05 * pos_bias_x[1];
	
	pos_out_x = pos_bias_x[1] * cpid.Kp + pi_x * cpid.Ki + (pos_bias_x[1] - pos_bias_x[0]) * cpid.Kd ;
	
        if(pos_out_x > cpid.limit)
		pos_out_x = cpid.limit;
	else if(pos_out_x < (-cpid.limit))
		pos_out_x = (-cpid.limit);
        if(pi_x > 0.3)
		pi_x = 0.3;
	if(pi_x < -0.3)
		pi_x = -0.3;

	return pos_out_x;	

}


float rmg2_Ypid_control(rmg2::rmg2_pid& cpid)
{
      float pos_out_y = 0;
	
	static float pi_y = 0;
	static float pos_bias_y[2] = {0.0 , 0.0 };
	
	pos_bias_y[0] = pos_bias_y[1];
	pos_bias_y[1] = cpid.target - cpid.feedback;

	if((pos_bias_y[1] < 0.02) && (pos_bias_y[1] > -0.02) )
		pos_bias_y[1] = 0;
	
	pi_y += 0.05 * pos_bias_y[1];
	
	pos_out_y = pos_bias_y[1] * cpid.Kp + pi_y * cpid.Ki + (pos_bias_y[1] - pos_bias_y[0]) * cpid.Kd ;
	
        if(pos_out_y > cpid.limit)
		pos_out_y = cpid.limit;
	else if(pos_out_y < (-cpid.limit))
		pos_out_y = (-cpid.limit);
        if(pi_y > 0.3)
		pi_y = 0.3;
	if(pi_y < -0.3)
		pi_y = -0.3;

	return pos_out_y;	

}


float rmg2_Zpid_control(rmg2::rmg2_pid& cpid)
{
      float pos_out_z = 0;
	
	static float pi_z = 0;
	static float pos_bias_z[2] = {0.0 , 0.0 };
	
	pos_bias_z[0] = pos_bias_z[1];
	pos_bias_z[1] = cpid.target - cpid.feedback;

	if((pos_bias_z[1] < 0.02) && (pos_bias_z[1] > -0.02) )
		pos_bias_z[1] = 0;
	
	pi_z += 0.05 * pos_bias_z[1];
	
	pos_out_z = pos_bias_z[1] * cpid.Kp + pi_z * cpid.Ki + (pos_bias_z[1] - pos_bias_z[0]) * cpid.Kd ;
	
        if(pos_out_z > cpid.limit)
		pos_out_z = cpid.limit;
	else if(pos_out_z < (-cpid.limit))
		pos_out_z = (-cpid.limit);
        if(pi_z > 0.3)
		pi_z = 0.3;
	if(pi_z < -0.3)
		pi_z = -0.3;

	return pos_out_z;	

}


float rmg2_Yawpid_control(rmg2::rmg2_pid& cpid)
{
      float pos_out_h = 0;
	
	static float pi_h = 0;
	static float pos_bias_h[2] = {0.0 , 0.0 };
	
	pos_bias_h[0] = pos_bias_h[1];
	pos_bias_h[1] = cpid.target - cpid.feedback;

	if((pos_bias_h[1] < 0.02) && (pos_bias_h[1] > -0.02) )
		pos_bias_h[1] = 0;
	
	pi_h += 0.05 * pos_bias_h[1];
	
	pos_out_h = pos_bias_h[1] * cpid.Kp + pi_h * cpid.Ki + (pos_bias_h[1] - pos_bias_h[0]) * cpid.Kd ;
	
        if(pos_out_h > cpid.limit)
		pos_out_h = cpid.limit;
	else if(pos_out_h < (-cpid.limit))
		pos_out_h = (-cpid.limit);
        if(pi_h > 0.3)
		pi_h = 0.3;
	if(pi_h < -0.3)
		pi_h = -0.3;

	return pos_out_h;	

}
class PID{
 public:
    PID():Kp(1.0),Kd(0.1),Ki(0.1),max_limit(0.8),min_limit(-0.8),errlast(0),errnow(0),integrate(0),out(0){}
    PID(float kp,float ki,float kd):Kp(kp),Kd(kd),Ki(ki){}
        
        float rmg2_pid_controller(rmg2::rmg2_pid& cpid)
	{
		errlast = errnow;
		errnow  = cpid.target - cpid.feedback;
		if((errnow < 0.005) && (errnow > -0.005))
			errnow = 0;
		integrate += 0.05*errnow;
	        out = Kp* errnow + Kd*(errnow - errlast) + Ki*integrate;
		
	     if(out > max_limit) out = max_limit;
	     if(out < min_limit) out = min_limit;
		if(integrate > 0.3)
			integrate = 0.3;
		if(integrate < -0.3)
			integrate = -0.3;
		return out;
				
	}	
/*	float rmg2_pid_controller(float value)
	{
	     integrate += value;
	     float inte = Ki*integrate/PID_RATE;
	     errnow = value;
	     if(inte > 0.2) inte = 0.2;
	     if(inte < -0.2) inte = -0.2;
	     out = Kp* errnow + Kd*( errnow - errlast) + inte;
	     errlast  = errnow;
	     if(out > max_limit) out = max_limit;
	     if(out < min_limit) out = min_limit;
	     return out;
	}
*/
	void pid_set_gains(float kp,float ki,float kd)
	{
		Kp = kp;
		Ki = ki;
 		Kd = kd;
	}
        float getValue()
	{
		return out;
	}
	void setValue(float value)
	{
	 out = value;
	}
	void setKp(float value)
	{
		Kp = value;
	}
	void setFeedback(float value)
	{
		Feedback = value;
	}
	void setTarget(float value)
	{
		Target = value;
	}
	void setParam(float kp,float ki,float kd)
	{
		Kp = kp;
		Ki = ki;
		Kd = kd;
	}
	void setLimit(float limit)
	{
		max_limit = limit;
		min_limit = -limit;
	}
	float getLimit(char choose)
	{
		if(choose == 1)
			return max_limit;
		else
			return -min_limit;
	}
	float getError()
	{
		return errnow;
	}
private:
	float Kp;
	float Ki;
	float Kd;
	float max_limit;
	float min_limit;
	float errlast;
	float errnow;
	float integrate;
	float Target;
	float Feedback;
	float out;
};
