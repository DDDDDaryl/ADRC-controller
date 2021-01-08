#ifndef __TRANSIENT_PROFILE_H
#define __TRANSIENT_PROFILE_H


//**********   TD Parameters ************
//float r=2000;
//float h=0.002;

float sign(float x);

float fst(float x1,float x2,float v, float r, float h);

float fal(float e,float alfa,float delta);

class transient_profile {
private:
	float td_x1;
	float td_x2;
	float td_r;
	float td_h;
    float td_h0;

public:
	transient_profile() : td_x1(0), td_x2(0), td_r(4), td_h(0.01), td_h0(0.02) {}
	~transient_profile() = default;
	
	void init(float r, float h) {
		td_r = r;
		td_h = h;
        td_h0 = 2 * td_h;
	}
	
	void init(float h) {
		td_h = h;
        td_h0 = 2 * td_h;
	}
    
	float iter(float v) {
		td_x1 = td_x1 + td_h * td_x2;
		td_x2 = td_x2 + td_h * fst(td_x1, td_x2, v, td_r, td_h0);
		return td_x1;
	}
	
	float get_td_x1() const {
		return td_x1;
	}
	
	float get_td_x2() const {
		return td_x2;
	}
};


#endif