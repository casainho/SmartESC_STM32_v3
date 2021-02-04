#include <arm_math.h>

static const uint16_t LUT_atan[101]={0,
		209,
		417,
		626,
		834,
		1042,
		1250,
		1458,
		1665,
		1872,
		2079,
		2285,
		2491,
		2697,
		2902,
		3106,
		3310,
		3513,
		3715,
		3917,
		4118,
		4318,
		4517,
		4716,
		4914,
		5110,
		5306,
		5501,
		5695,
		5888,
		6080,
		6271,
		6461,
		6649,
		6837,
		7023,
		7209,
		7393,
		7576,
		7757,
		7938,
		8117,
		8295,
		8472,
		8647,
		8821,
		8994,
		9165,
		9336,
		9504,
		9672,
		9838,
		10003,
		10167,
		10329,
		10490,
		10649,
		10807,
		10964,
		11119,
		11274,
		11426,
		11578,
		11728,
		11876,
		12024,
		12170,
		12314,
		12458,
		12600,
		12740,
		12880,
		13018,
		13154,
		13290,
		13424,
		13557,
		13688,
		13819,
		13948,
		14076,
		14202,
		14328,
		14452,
		14575,
		14696,
		14817,
		14936,
		15054,
		15171,
		15287,
		15402,
		15515,
		15628,
		15739,
		15849,
		15958,
		16066,
		16173,
		16279,
		16383
} ;

q31_t atan2_LUT(q31_t e_alpha, q31_t e_beta){

        uint8_t index =0;
        uint8_t frac =0;
        q31_t angle_obs=0;
        uint8_t i=0;
        //Quadrant 1 +
        if (e_alpha>0 && e_beta>0){
            if(e_alpha>e_beta){  // y/x < 1

             index = (e_beta*100)/e_alpha;
             frac = (e_beta*1000/e_alpha)-10*index;
             angle_obs=LUT_atan[index]+(LUT_atan[index+1]-LUT_atan[index])*frac/10; //interpolate between to values
            }
             else {                     // y/x > 1 artan(y/x) = Pi/2 - artan(x/y)

             index = (e_alpha*100)/e_beta;
             frac = (e_alpha*1000/e_beta)-10*index;
                 angle_obs= 32767-(LUT_atan[index]+((LUT_atan[index+1]-LUT_atan[index])*frac)/10);
             }
        }
        //Quadrant 2
        if (e_alpha<0 && e_beta>0){

	        if(-e_alpha>e_beta){  // y/x < 1
            index = (e_beta*100)/-e_alpha;
            frac = (e_beta*1000/-e_alpha)-10*index;
            angle_obs = 65535-(LUT_atan[index]+(LUT_atan[index+1]-LUT_atan[index])*frac/10); //interpolate between to values
           }
            else {                     // y/x > 1 artan(y/x) = Pi/2 - artan(x/y)

            index = (-e_alpha*100)/e_beta;
            frac = (-e_alpha*1000/e_beta)-10*index;
                angle_obs= 65535-(32767-(LUT_atan[index]+((LUT_atan[index+1]-LUT_atan[index])*frac)/10));
            }

        }

        //Quadrant 3
        if (e_alpha<0 && e_beta<0){
            if(-e_alpha>-e_beta){  // y/x < 1

             index = (e_beta*100)/e_alpha;
             frac = (e_beta*1000/e_alpha)-10*index;
             angle_obs= -65535 + LUT_atan[index]+(LUT_atan[index+1]-LUT_atan[index])*frac/10; //interpolate between to values
            }
             else {                     // y/x > 1 artan(y/x) = Pi/2 - artan(x/y)

             index = (e_alpha*100)/e_beta;
             frac = (e_alpha*1000/e_beta)-10*index;
                 angle_obs= -32767-(LUT_atan[index]+((LUT_atan[index+1]-LUT_atan[index])*frac)/10);
             }

        }

        //Quadrant 4
        if (e_alpha>0 && e_beta<0){
            if(e_alpha>-e_beta){  // y/x < 1

             index = (-e_beta*100)/e_alpha;
             frac = (-e_beta*1000/e_alpha)-10*index;
             angle_obs=-(LUT_atan[index]+(LUT_atan[index+1]-LUT_atan[index])*frac/10); //interpolate between to values
            }
             else {                     // y/x > 1 artan(y/x) = Pi/2 - artan(x/y)

             index = (e_alpha*100)/-e_beta;
             frac = (e_alpha*1000/-e_beta)-10*index;
                 angle_obs= -(32767-(LUT_atan[index]+((LUT_atan[index+1]-LUT_atan[index])*frac)/10));
             }

        }
        return ((angle_obs<<15)+1550960412); //angle in degree to q31 Look up table is scaled to 90° = 2^16 -1431655765
}
