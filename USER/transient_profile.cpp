#include "transient_profile.h"
#include <cmath>

using namespace std;

float sign(float x)
{
  if(x>0)
      return(1.0f);
  if(x<0)
      return(-1.0f);
  else return 0.0f;
}

float fst(float x1,float x2,float v, float r, float h)
{
	float td_y=0;
	float a0=0;
	float a=0;
	float fhan=0;
	float d=0;
	float d0=0;
	//float flag_y=0;
	//float flag_a=0;
	
	d=r*h;
	d0=h*d;
	td_y=x1-v+h*x2;
	a0=sqrt(d*d+8*r*fabs(td_y));
	//if(td_y>0) flag_y=1;           //sign(td_y);
	//else 	   flag_y=-1;
	
	if(fabs(td_y)>d0)
		a=x2+0.5*(a0-d)*sign(td_y);
	else
		a=x2+td_y/h;
		
	//if(a>0) flag_a=1;
	//else 	flag_a=-1;	
	if (fabs(a)>d)
		fhan=-r*sign(a);
	else
		fhan=-r*a/d;
	return(fhan);
}

float fal(float e,float alfa,float delta)
{
	//float flag_e=0.0;
	float y=0.0;
	//if(e>0) flag_e=1.0;
	//if(e<0) flag_e=-1.0;
	//if(e==0) flag_e=0.0;

	if(fabs(e)>delta) y=pow(fabs(e),alfa)*sign(e);
	else			  y=e/pow(delta,1.0-alfa);
	return(y);	
}
