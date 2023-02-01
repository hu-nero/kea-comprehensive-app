#include "Read_Res_Val.h"

uint8_t IIC_InaR_TxData[4] = {0};
uint8_t IIC_InaR_RdData[4] = {0};

uint16_t RES_CAL1 = _RES_CAL1_VALUE;
uint16_t RES_CAL2 = _RES_CAL2_VALUE;

uint8_t ResMeasure_Switch = 1;
uint8_t ResMeasure_Sta = 1;

uint16_t INA226_R_CFGBuf = 0;
uint8_t ResMeasureCheckTimer = 0;
volatile uint32_t HV2              = 10000;          /*  HV2�������ĵ�ѹֵ           */

uint16_t R_HV2_To_GND  = 0xFFFF;          /*  HV2��GND�ľ�Ե����ֵ        */
uint16_t R_FLQ_To_GND  = 0xFFFF;          /*  ��������GND�ľ�Ե����ֵ    */
uint16_t UD = 0;

int16_t  R_Samp[482];           /*��Ե����������ݲɼ��������� */
uint16_t LastsampTime  = 1000 ;
int32_t time           = 0;
uint8_t ON_Negative    = 1;
volatile int16_t Rprocess_Samp[30];
volatile uint8_t R_Step             = 0;           /*  ��Ե����������� */
volatile uint8_t R_Finsh            = 0;           /*  ��Ե����������ݲɼ���ɱ�־ */
volatile uint8_t R_TestMethod       = 0;
volatile uint8_t LessSampTime       = 0;           /*  �ӳ���Ե����������ݲɼ�ʱ���־ */
volatile uint8_t R_MeasureTime      =40;          /*  ��Ե�������ʱ����    */
volatile uint8_t RSampleNum         = 0;
volatile int16_t R_Result           = 0;

uint8_t RP_ResultNum = 0;
uint8_t RN_ResultNum = 0;
uint16_t R_HV2_To_GND_Samp[10] = {5000,5000,5000,5000,5000,5000,5000,5000,5000,5000};
uint16_t R_FLQ_To_GND_Samp[10] = {5000,5000,5000,5000,5000,5000,5000,5000,5000,5000};


volatile uint8_t R_Turn             = 0;
volatile uint8_t SYS_Turn           = 0;
volatile uint8_t nosampflag         = 0;

volatile uint32_t HV2_in            = 10000;          /*  �����ڲ�ʹ�õĵ�ѹֵ  */
volatile uint16_t R_next            = 0;          /*  ��Ե��������м����        */
volatile uint16_t SampTime          = 110;        /*  ��Ե����������ݲɼ�ʱ��  */
volatile int16_t SysCharactTime_ms  = 0;          /*  ϵͳ��ʵ�ֳ���T             */

volatile int32_t R_HV2threL         = 0;          /*  һ�ξ�Ե�������������ʱ�����ѹ������ֵ�趨    */
volatile int32_t R_HV2threR         = 0;          /*  һ�ξ�Ե�������������ʱ�����ѹ������ֵ�趨    */
volatile int32_t UHV2_To_FLQ        = 0;          /*  HV2�Է������ĵ�ѹֵ       */
volatile int32_t UHV2_To_GND        = 0;

volatile int32_t UFLQ_To_GND        = 0;          /*  ��Ե��������м���������࿪�ش�ʱ�Եصĵ�ѹֵ   */

uint16_t HVRH = 0;
uint16_t HVRL = 0;
uint16_t HVR = 0;
uint16_t RvalueTemp = 0;
uint16_t RvalueVp = 932;
uint16_t RvalueVl = 930;
//uint16_t RvalueVp = 5730;//#6
uint16_t HVRHNODE = 0;

uint16_t CalibVTotal_H = 2302;
uint16_t CalibVTotal_L = 80;//240 

uint16_t CalibVinsHip_H = 20099;
uint16_t CalibVinsHip_L = 80;


const uint16_t Tab_Log[101] = {6666, 4605, 3912 ,3507, 3219, 2996, 2813, 2659, 2526, 2408,2303, 2207, \
        2120,2040,1966,1897,1833,1772,1715,1661,1609,1561,1514,1470,1427,1386,1347,1309,1273,1238,1204,1171,1139, \
        1109,1079,1050,1022,994,968,942,916,892,868,844,821,799,777,755,734,713,693,673,654,635,616,598,580,562,  \
        545,528,511,494,478,462,446,431,416,400,386,371,357,342,329,315,301,288,274,261,248,236,223,211,198,186,  \
        174,163,151,139,128,117,105,94,83,73,62,51,41,30,20,10};

const uint16_t Tab2[161]    = {1 , 2 , 4 , 5 , 7 , 8 , 10, 11, 13, 14, 16, 17, 19, 20, 22, 23, 25, 26, \
                          28, 29, 31, 32, 34, 35, 37, 38, 40, 41, 43, 44, 46, 47, 49, 50, 52, 53, \
                          55, 56, 58, 59, 61, 62, 64, 65, 67, 68, 70, 71, 73, 74, 76, 77, 79, 80, 82,\
                          83, 85, 86, 88, 89, 91, 92, 94, 95, 97, 98, 100,101, 103, 104, 106, 107, 109, \
                          110, 112, 113, 115, 116, 118, 119, 121, 122, 124, 125, 127, 128, 130, 131, 133, \
                          134, 136, 137, 139, 140, 142, 143, 145, 146, 148, 149, 151, 152, 154, 155, 157, 158,\
                          160, 161, 163, 164, 166, 167, 169, 170, 172, 173, 175, 176, 178, 179, 181, 182,\
                          184, 185, 187, 188, 190, 191, 193, 194, 196, 197, 199, 200, 202, 203, 205, 206, \
                          208, 209, 211, 212, 214, 215, 217, 218, 220, 221, 223, 224, 226, 227, 229, 230,\
                          232, 233, 235, 236, 238, 239, 241};   /* ѡ����Ҫ��������ݵĵ�ַ�б�        */


const uint16_t Tab3[161]    = {0,  3 , 6 , 9 , 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54,
                          57, 60, 63, 66, 69, 72, 75, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108,
                          111, 114, 117, 120, 123, 126, 129, 132, 135, 138, 141, 144, 147, 150, 153,
                          156, 159, 162, 165, 168, 171, 174, 177, 180, 183, 186, 189, 192, 195, 198,
                          201, 204, 207, 210, 213, 216, 219, 222, 225, 228, 231, 234, 237, 240, 243,
                          246, 249, 252, 255, 258, 261, 264, 267, 270, 273, 276, 279, 282, 285, 288,
                          291, 294, 297, 300, 303, 306, 309, 312, 315, 318, 321, 324, 327, 330, 333,
                          336, 339, 342, 345, 348, 351, 354, 357, 360, 363, 366, 369, 372, 375, 378,
                          381, 384, 387, 390, 393, 396, 399, 402, 405, 408, 411, 414, 417, 420, 423,
                          426, 429, 432, 435, 438, 441, 444, 447, 450, 453, 456, 459, 462, 465, 468,
                          471, 474, 477, 480 };   /* ѡ����Ҫ��������ݵĵ�ַ�б�        */



uint8_t INA226_R_SendData(uint8_t *pdata, uint8_t len) {
	return IIC1_Transmit(_INA226_R_ADDR, pdata, len);
}

uint8_t INA226_R_ReadData(uint8_t *pdata, uint8_t len) {
	return IIC1_Receive(_INA226_R_ADDR, pdata, len);
}

uint8_t INA226_R_Init(void) {
  	uint8_t err = 0;
	IIC_InaR_TxData[0] = 0;
	IIC_InaR_TxData[1] = 0x41;
	IIC_InaR_TxData[2] = 0x67;
	err = INA226_R_SendData(IIC_InaR_TxData, 3);
	return err;
}

uint8_t INA226_R_GetRegData(uint8_t reg, uint16_t *rdata) {
	uint8_t err = 0;
	uint16_t rdatabuf = 0;
	IIC_InaR_TxData[0] = reg;
	memset(IIC_InaR_RdData, 0, sizeof(IIC_InaR_RdData));
	err |= INA226_R_SendData(IIC_InaR_TxData, 1);
	err |= INA226_R_ReadData(IIC_InaR_RdData, 2);
	rdatabuf |= (uint16_t)(((uint16_t)IIC_InaR_RdData[0])<<8);
	rdatabuf |= (uint16_t)(((uint16_t)IIC_InaR_RdData[1]));
	rdata[0] = rdatabuf;
	return err;
}

/**********************************************************
**  ����: GetVinsTotal
**  ����: ���߱߾�Ե�����ѹ
**  ����: *inVdata  ��ѹָ��	
**  ����ֵ: 0 OK�� >1 err 
***********************************************************/    
uint8_t GetVinsTotal(uint16_t *inVdata){
  uint8_t err =0;
  static uint16_t ownVdata = 0;
  err = INA226_R_GetRegData(BusVol_Reg, &INA226_R_CFGBuf);;  
  inVdata[0] = (uint32_t)ownVdata*CalibVTotal_H/CalibVTotal_L; //�궨 
  return  err;
}


/**********************************************************
**  ����: GetVinsHp 
**  ����: ���߱߾�Ե�����ѹ Insulation High Part V
**  ����: *inVdata  ��ѹָ��	
**  ����ֵ: 0 OK�� >1 err 
***********************************************************/    
uint8_t GetVinsHp(uint16_t *inVdata){
  uint8_t err =0;
  uint16_t ownVdata = 0; 
  uint16_t harfV=0;
  
  //err = GetVbus( &ownVdata );  
  
  if( err >0)  //�д�ֱ�ӳ�ȥ ��ѹ����֮ǰ�ĵ�ѹ
       return err;
 
  harfV = (uint32_t)ownVdata*CalibVinsHip_H/CalibVinsHip_L; //�궨 
 
  inVdata[0] = harfV;
  return  err;
}
/**********************************************************  
**  ����:  GetVinsLp
**  ����: ���ױ߾�Ե�����ѹ   Insulation Low Part V
**  ����: *inVdata  ��ѹָ��	
**  ����ֵ:0 OK�� 1��2��3 err   255����ͬ���ܺ�������ִ�� �����˲ɼ�����	   
***********************************************************/   
uint8_t GetVinsLp(uint16_t *inVdata){
  uint8_t err =0;
  int16_t ownCdata;
 
   //err = MINA_GetCURData( &ownCdata ); 
     
  if( err >0){ //�д�ֱ�ӳ�ȥ  ֵ����
       return err;
  }
  ownCdata = ((int32_t)ownCdata*25)/1000;   //��λ0.1mV  
  //ownCdata = (int32_t)ownCdata*2000820/820/100;    //����Ȼ���GND_COM��ѹ ��λ0.01V
  ownCdata = (int32_t)ownCdata*875/41;   //��ʱ��ֱ���ñ궨ϵ��
    
  inVdata[0] = ownCdata; 
  
  return  err;
}
 

int16_t GetRValue(void){
	
      int16_t RtempValue = 0;
      uint8_t err = 0;

      HVR = (uint16_t)HV2_in;

	if(R_Step == 1){
		err =  INA226_R_GetRegData(Shunt_Reg, &RvalueTemp);  //Vp��ѹ
		if(err == 0){
			HVRH =  RvalueTemp;
			HVRHNODE = ((uint32_t)HVRH*25*200/RvalueVp)+((uint32_t)HVRH*25/100000);//��λ10mV
			if( HVR <HVRHNODE){
				HVRH = 1;
			}
			else{
				HVRH = HVR - HVRHNODE;
			}
			HVRH = (HVRH/200)+HVRH;
		}else
		{}
		RtempValue = (int16_t)HVRH;
	}
	else if(R_Step == 2){
		//mdelay(5);
		// INA226_R_GetRegData(Shunt_Reg, &HVR);
		//HVR = ((uint32_t)HVR*25*220/RvalueVp)+((uint32_t)HVR*25/100000);
		RtempValue = (int16_t)HVR;///����д�Ĺ̶�ֵ �����ĳ�HVR = HV2 ����NA226_R_GetRegData(Shunt_Reg, &HVR)���� ��λ10mV
		}
	else if(R_Step == 3){
		err =  INA226_R_GetRegData(BusVol_Reg, &RvalueTemp);
		if(err == 0 ){
			HVRL = RvalueTemp;
			HVRL = ((uint32_t)HVRL*25352)/1000;//��λ10mV
		}
		 else{}
		RtempValue = HVRL;
	}
	else if(R_Step == 4)
	{}
    if(RtempValue<0){
      RtempValue = 1;
     }
	return RtempValue;
}

void RSamp(unsigned int samptime)
{
    long t = 0;
    static int16_t r_temp[3] = {0, 0, 0 }; /*  �����ɼ���������, �˲�ʹ��              */

    if(R_next < 9){
    }
    else if(R_next < 12){

        //r_temp[R_next - 9] =  GetRValue();
     	if(RSampleNum<30){
    		Rprocess_Samp[RSampleNum] = GetRValue();
    		RSampleNum++;
    		R_next--;
    	}
    	else if(RSampleNum == 30){

    		RSampleNum = 0;

    		R_Result = (Rprocess_Samp[0]+Rprocess_Samp[1]+Rprocess_Samp[2]+Rprocess_Samp[3]+Rprocess_Samp[4]+Rprocess_Samp[5]
			   +Rprocess_Samp[6]+Rprocess_Samp[7]+Rprocess_Samp[8]+Rprocess_Samp[9]+Rprocess_Samp[10]+Rprocess_Samp[11]
		           +Rprocess_Samp[12]+Rprocess_Samp[13]+Rprocess_Samp[14]+Rprocess_Samp[15]+Rprocess_Samp[16]+Rprocess_Samp[17]
			   +Rprocess_Samp[18]+Rprocess_Samp[19]+Rprocess_Samp[20]+Rprocess_Samp[21]+Rprocess_Samp[22]+Rprocess_Samp[23]
	                   +Rprocess_Samp[24]+Rprocess_Samp[25]+Rprocess_Samp[26]+Rprocess_Samp[27]+Rprocess_Samp[28]+Rprocess_Samp[29])/30;

    		r_temp[R_next - 9] = R_Result;


    	}

    }
    else if(R_next < 13){
         /*  ���ߵĵ�һ���㱣����R_Samp[Tab2[0]]�� */
        if(ON_Negative){
            R_Samp[Tab2[0]]  = (r_temp[0] + r_temp[1] + r_temp[2]) / 3;  //R_Samp[1]���һ����

        }else{
            R_Samp[Tab2[0]]  = -(r_temp[0] + r_temp[1] + r_temp[2]) / 3;
        }

        if(R_Samp[Tab2[0]] < 0){
            R_Samp[Tab2[0]] = 0;
        }
        if(R_Step == 2){
            R_Samp[Tab3[0]] = R_Samp[Tab2[0]]; /*  ������ڲ����ܵ�ѹʱ����Ҫ�������������ܵ�ѹ������R_Samp[0]��  */
            R_next = 15;           /*  һ����С��15��ֵ���˳��ܵ�ѹ�Ĳ���   */
        }
    }
    else if(R_next > 45){
        if((R_next > (samptime - 62)) && (R_next < (samptime - 58))){ //  ����48��С��52

     	if(RSampleNum<30){
    		Rprocess_Samp[RSampleNum] = GetRValue();
    		RSampleNum++;
    		R_next--;
    	}
    	else if(RSampleNum == 30){

    		RSampleNum = 0;

    		R_Result = (Rprocess_Samp[0]+Rprocess_Samp[1]+Rprocess_Samp[2]+Rprocess_Samp[3]+Rprocess_Samp[4]+Rprocess_Samp[5]
			   +Rprocess_Samp[6]+Rprocess_Samp[7]+Rprocess_Samp[8]+Rprocess_Samp[9]+Rprocess_Samp[10]+Rprocess_Samp[11]
		           +Rprocess_Samp[12]+Rprocess_Samp[13]+Rprocess_Samp[14]+Rprocess_Samp[15]+Rprocess_Samp[16]+Rprocess_Samp[17]
			   +Rprocess_Samp[18]+Rprocess_Samp[19]+Rprocess_Samp[20]+Rprocess_Samp[21]+Rprocess_Samp[22]+Rprocess_Samp[23]
	                   +Rprocess_Samp[24]+Rprocess_Samp[25]+Rprocess_Samp[26]+Rprocess_Samp[27]+Rprocess_Samp[28]+Rprocess_Samp[29])/30;

    		//r_temp[R_next - 9] = R_Result;
            r_temp[R_next - (samptime - 61)] = R_Result;

    	}       
                          

          //r_temp[R_next - (samptime - 61)] = GetRValue();
             ////�ڶ���������3��

        }else if(R_next < (samptime - 57)){///////////////////////////////С��53
            /*  �����ɼ�3�������һ��ƽ���˲�,Ȼ��洢������������  */
            if(ON_Negative){
                R_Samp[(samptime / 100) * 3 - 1]  = (r_temp[0] + r_temp[1] + r_temp[2]) / 3;//R_Samp[2]��ڶ�����
            }else{
                R_Samp[(samptime / 100) * 3 - 1]  = -(r_temp[0] + r_temp[1] + r_temp[2]) / 3;
            }
        }else if((R_next > (samptime - 22)) && (R_next < (samptime - 18))){// ����88��С��92
                  
          
     	if(RSampleNum<30){
    		Rprocess_Samp[RSampleNum] = GetRValue();
    		RSampleNum++;
    		R_next--;
    	}
    	else if(RSampleNum == 30){

    		RSampleNum = 0;

    		R_Result = (Rprocess_Samp[0]+Rprocess_Samp[1]+Rprocess_Samp[2]+Rprocess_Samp[3]+Rprocess_Samp[4]+Rprocess_Samp[5]
			   +Rprocess_Samp[6]+Rprocess_Samp[7]+Rprocess_Samp[8]+Rprocess_Samp[9]+Rprocess_Samp[10]+Rprocess_Samp[11]
		           +Rprocess_Samp[12]+Rprocess_Samp[13]+Rprocess_Samp[14]+Rprocess_Samp[15]+Rprocess_Samp[16]+Rprocess_Samp[17]
			   +Rprocess_Samp[18]+Rprocess_Samp[19]+Rprocess_Samp[20]+Rprocess_Samp[21]+Rprocess_Samp[22]+Rprocess_Samp[23]
	                   +Rprocess_Samp[24]+Rprocess_Samp[25]+Rprocess_Samp[26]+Rprocess_Samp[27]+Rprocess_Samp[28]+Rprocess_Samp[29])/30;

    		//r_temp[R_next - 9] = R_Result;
            r_temp[R_next - (samptime - 21)] = R_Result;

    	}       
                      
          
          //r_temp[R_next - (samptime - 21)] =  GetRValue();


        }else if(R_next < (samptime - 17)){
            /*  �����ɼ�3�������һ��ƽ���˲�,Ȼ��洢������������  */
            if(ON_Negative){
                R_Samp[(samptime / 100) * 3]  = (r_temp[0] + r_temp[1] + r_temp[2]) / 3;//R_Samp[3]���������
            }else{
                R_Samp[(samptime / 100) * 3]  = -(r_temp[0] + r_temp[1] + r_temp[2]) / 3;
            }
        }else if((R_next > (samptime - 12)) && (R_next < (samptime - 8))){ // ����98��С��102


          // r_temp[R_next - (samptime - 11)] =  GetRValue();

           
     	if(RSampleNum<30){
    		Rprocess_Samp[RSampleNum] = GetRValue();
    		RSampleNum++;
    		R_next--;
    	}
    	else if(RSampleNum == 30){

    		RSampleNum = 0;

    		R_Result = (Rprocess_Samp[0]+Rprocess_Samp[1]+Rprocess_Samp[2]+Rprocess_Samp[3]+Rprocess_Samp[4]+Rprocess_Samp[5]
			   +Rprocess_Samp[6]+Rprocess_Samp[7]+Rprocess_Samp[8]+Rprocess_Samp[9]+Rprocess_Samp[10]+Rprocess_Samp[11]
		           +Rprocess_Samp[12]+Rprocess_Samp[13]+Rprocess_Samp[14]+Rprocess_Samp[15]+Rprocess_Samp[16]+Rprocess_Samp[17]
			   +Rprocess_Samp[18]+Rprocess_Samp[19]+Rprocess_Samp[20]+Rprocess_Samp[21]+Rprocess_Samp[22]+Rprocess_Samp[23]
	                   +Rprocess_Samp[24]+Rprocess_Samp[25]+Rprocess_Samp[26]+Rprocess_Samp[27]+Rprocess_Samp[28]+Rprocess_Samp[29])/30;

            r_temp[R_next - (samptime - 11)] = R_Result;

    	}       
                   
          

        }else if(R_next < (samptime - 7)){
            /*  �����ɼ�3�������һ��ƽ���˲�,Ȼ��洢������������  */
            if(ON_Negative){
                R_Samp[(samptime / 100) * 3 + 1] = (r_temp[0] + r_temp[1] + r_temp[2]) / 3;//R_Samp[4]����ĸ���
            }else{
                R_Samp[(samptime / 100) * 3 + 1] = -(r_temp[0] + r_temp[1] + r_temp[2]) / 3;
            }

        }
    }
}




long FindLog(unsigned int x) {
    long result=0;
    if(x > 99)
    	x = 99;
    result =  (long)Tab_Log[x];
    return result;

}


void RProcess(void)
{
    long               ud = 0, u = 0,u1 = 0, u2 = 0, u3 = 0, m1 = 0, m2 = 0;
    uint16_t FastCaculateU1 = 0, FastCaculateU2 = 0,FastCaculateU3 = 0,FastCaculateU4 = 0;
    uint16_t D_value1 = 0,D_value2 = 0,D_value3 = 0;
    uint32_t        r_hv2_to_gnd = 5000;
    uint32_t        r_flq_to_gnd = 5000;
    static uint16_t r_hv2_to_gnd_save = 5000;
    static uint16_t r_flq_to_gnd_save = 5000;

    if(R_Finsh){
        R_Finsh = 0;
        UHV2_To_FLQ =  R_Samp[Tab3[0]];      /*  ��ȡ�ܵ�ѹ              */
        if(UHV2_To_GND < 1)
        {
        	UHV2_To_GND = 1;
        }
        if(UFLQ_To_GND < 1)
        {
        	UFLQ_To_GND = 1;
        }

        ud = UHV2_To_FLQ - UHV2_To_GND - UFLQ_To_GND;
        UD = ud;
        if(ud < 1){
            ud = 1;
        }
        r_hv2_to_gnd = (ud * 2010) / UFLQ_To_GND;  /*  �������Ե����ֵ  */
        r_flq_to_gnd = (ud * 2010) / UHV2_To_GND;

        if(r_hv2_to_gnd > 5000){
          r_hv2_to_gnd = 5000;
          R_HV2_To_GND = (uint16_t)(r_hv2_to_gnd);
        }
        else if(r_hv2_to_gnd > 200){
        	R_HV2_To_GND_Samp[RP_ResultNum] =  (unsigned int)(r_hv2_to_gnd);
            RP_ResultNum++;
            if(RP_ResultNum > 9){
           	 RP_ResultNum = 0;
            }
            R_HV2_To_GND = (R_HV2_To_GND_Samp[0]+R_HV2_To_GND_Samp[1]+R_HV2_To_GND_Samp[2]+R_HV2_To_GND_Samp[3]+R_HV2_To_GND_Samp[4]+
            		R_HV2_To_GND_Samp[5]+R_HV2_To_GND_Samp[6]+R_HV2_To_GND_Samp[7]+R_HV2_To_GND_Samp[8]+R_HV2_To_GND_Samp[9])/10;

        }
        else{
          R_HV2_To_GND = (uint16_t)(r_hv2_to_gnd);
        }


        if(r_flq_to_gnd  > 5000){
          r_flq_to_gnd  = 5000;
          R_FLQ_To_GND = (uint16_t)(r_flq_to_gnd);
        }
        else if(r_flq_to_gnd  > 200){
        R_FLQ_To_GND_Samp[RN_ResultNum] =  (unsigned int)(r_flq_to_gnd);
        RN_ResultNum++;
        if(RN_ResultNum > 9){
       	 RN_ResultNum = 0;
        }
        R_FLQ_To_GND = (R_FLQ_To_GND_Samp[0]+R_FLQ_To_GND_Samp[1]+R_FLQ_To_GND_Samp[2]+R_FLQ_To_GND_Samp[3]+R_FLQ_To_GND_Samp[4]+
        		R_FLQ_To_GND_Samp[5]+R_FLQ_To_GND_Samp[6]+R_FLQ_To_GND_Samp[7]+R_FLQ_To_GND_Samp[8]+R_FLQ_To_GND_Samp[9])/10;

       }
        else{
          R_FLQ_To_GND = (uint16_t)(r_flq_to_gnd);
        }
       // R_HV2_To_GND = (uint16_t)(r_hv2_to_gnd);
       // R_FLQ_To_GND = (uint16_t)(r_flq_to_gnd);
        if(R_TestMethod){
            SYS_Turn = 1; //���ٲ�����ɺ�ͷ��ͳ�ȥ
            R_TestMethod = 0;//  ��һ�ο��ٲ�������ͽ�����������ģʽ
            //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            R_MeasureTime = 40;
            //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        }
    }else{
        if(R_TestMethod){     //  ���ٲ���
 
        }else{
            if(R_next < (SampTime - 6)) {
                RSamp(SampTime);
            }else if(R_next < (SampTime - 5)) {
                u1 =  R_Samp[Tab2[0]];               /*  ��ȡ�����ϵ�һ���ѹ R_Samp[1]   */
                u2 =  R_Samp[Tab2[SampTime / 100]];  /*  ��ȡ�����ϵڶ����ѹ R_Samp[2]   */
                u3 =  R_Samp[Tab3[SampTime / 100]];  /*  ��ȡ�����ϵ������ѹ R_Samp[3]  */

                if(u2 > u1) u2 = u1;
                if(u3 > u2) u3 = u2;

                m1 = (u1 * u3) - (u2 * u2);
                m2 =  u1 + u3 - u2 - u2;
                if(m1 < 0){
                  
                    m1 = 0;
                }

                if( LastsampTime < 600)
                {
                    LastsampTime = 600;
                }
                else if(LastsampTime > 15000)
                {
                    LastsampTime = 15000;
                }

                if( SampTime < LastsampTime - 300)
                {
                    LessSampTime = 1;
                }
                if(m2 < 1) {                /*  m2��С���������ֿ���   */
                    if(((u2 - u3) > 100) && (SampTime < 16000)){    /* SampTime < 16000 �ɼ��������߽�����һ��бֱ�ߣ����������յ㻹�кܳ���ʱ�䣬��Ҫ��������  */
                        LessSampTime = 1;   /*  ��λ����������־λ   */
                    }else{                  /*  �ɼ�����������һ��ƽֱ��, ����ϵͳʱ�䳣����С���ɼ��������ߵ��յ�ֵ  */
                        u = u3 - 1;             /*  ʹ�����ɼ����ĵ���Ϊ���ߵ��յ�ֵ  */
                        if(u < 0){
                            u = 0;
                        }
                        SysCharactTime_ms = 0;
                    }
                }else{
                    u = m1 / m2;     /*  ��������Ԥ��������ֵ̬  */

                    if(u > u3){
                        u = u3 - 1;
                    }
                    if(u == u1) {
                        SysCharactTime_ms = 0;
                    }else{
                        time =((u2 - u)*100) / (u1 - u);
                        SysCharactTime_ms = ((long)(SampTime - 30) * 250) / FindLog(((u2 - u)*100) / (u1 - u));
                    }
                }

                if((SysCharactTime_ms < 29) && (SampTime > 300) && (!LessSampTime)){
                    if(R_Step == 1){
                        UHV2_To_GND = u;

                    }else if(R_Step == 3){
                        UFLQ_To_GND = u;
                    }
                    R_next = SampTime;
                    LastsampTime =  SampTime;

                }else if((SysCharactTime_ms < ((SampTime - 20) / 3)) && (!LessSampTime) && (SampTime > 300)){
                    if(SysCharactTime_ms < 29){
                        SysCharactTime_ms = 0;
                    }else if(SysCharactTime_ms < 45){
                        SysCharactTime_ms = SysCharactTime_ms - 29;
                    }else {
                        SysCharactTime_ms = 16;
                    }//��ϵͳʱ�䳣����һ��0��16
                    /*  ���������ֵ̬�������ϲ����������һ�����ϣ���֤�������ֵ����ƫ����� */
                    u = SysCharactTime_ms * u + (16 - SysCharactTime_ms) * u3;
                    u = u >> 4;

                    if(R_Step == 1){
                        UHV2_To_GND = u;
                    }else if(R_Step == 3){
                        UFLQ_To_GND = u;
                    }
                    R_next = SampTime;
                    LastsampTime =  SampTime;
                } else{
                    SampTime += 100;
                    LessSampTime = 0;
                }
            }
        }
    }
}


void R_Measure(uint8_t R_cnt,uint32_t HVIN){
      
           HV2_in = HVIN;
          // HVR = HVIN;//���Դ���
           if(R_Turn){
             R_Turn = 0;
             RProcess();
             }
           if((R_MeasureTime == 0) || (HV2_in < 5000)){     ////��HV2�ļ��㺯����Ҫ��HV2��ֵ����HV2_in  //  �����HV2_IN ��Ҫ��HV2ֵһ��
              R_Step          = 0;
              R_HV2_To_GND    = 0xFFFF;
              R_FLQ_To_GND    = 0xFFFF;

             _RP_CTL_OFF;
             _RN_CTL_OFF;
              
          }
          else if(R_cnt > R_MeasureTime){
              ResMeasureCheckTimer = 0;
              
              if(R_Step == 0){
                  R_Step      = 1;
                  SampTime    = 110;
                  R_next      = 0;
                  R_HV2threL   = HV2_in - (HV2_in >> 4);//�����ѹ�仯Ϊ�ܵ�ѹ�ġ�6.25%
                  R_HV2threR   = HV2_in + (HV2_in >> 4);
              }
          } 
           if((HV2_in > R_HV2threR) || (HV2_in < R_HV2threL)){ //�ܵ�ѹ��⣬����б仯�������²���
              R_Step = 0;
          } 
  
          if(R_Step == 1){
              R_next++;
              if(R_next ==1){

             
             _RP_CTL_ON;

             _RN_CTL_OFF; 
                             
                  nosampflag = 4;//  ���عضϣ�4����ѹ�����ɼ����ڣ�6ms���ڽ�ֹ�ɼ���ѹ�͵���

              }
              else if(R_next < 8){

              }
              else if(R_next < SampTime){
                  R_Turn = 1;

              }
              else {
                  R_Step = 2;
                  SampTime = 110;
                  R_next = 0;
              }
          }
          else if(R_Step == 2){
              R_next++;
              if(R_next == 1){

                
             _RP_CTL_OFF;
             _RN_CTL_OFF;
                
                  nosampflag = 4;//  ���عضϣ�4����ѹ�����ɼ����ڣ�6ms���ڽ�ֹ�ɼ���ѹ�͵���


              }else if(R_next < 8){

              }else if(R_next < 15){
                  R_Turn = 1;
              }else {
                  R_Step = 3;
                  SampTime = 110;
                  R_next = 0;
              }
          }else if(R_Step == 3){
              R_next++;

              if(R_next == 1){


             //_RP_CTL_ON;
             _RN_CTL_ON;
             _RP_CTL_OFF;
             //_RN_CTL_OFF;                
                

                  nosampflag = 4;//  ���عضϣ�4����ѹ�����ɼ����ڣ�6ms���ڽ�ֹ�ɼ���ѹ�͵���

              }else if(R_next <8){
              }else if(R_next < SampTime){
                  R_Turn = 1;
              }else {
                  R_Turn = 1;
                  R_next = 0;
                  R_Finsh = 1;
                  R_Step = 0;


               _RP_CTL_OFF;
               _RN_CTL_OFF;
                      
                  nosampflag = 4;//  ���عضϣ�4����ѹ�����ɼ����ڣ�6ms���ڽ�ֹ�ɼ���ѹ�͵���
              }
          }
}

/**********************************************************
**  ����:     Res_Measure_Interface 
**
**  ����:    ����Ե����
**
**  ����:    uint8_t Timer_Counter         ��ʱ���������ݶ�ʱ����λʱ�������������
**                                         �����Զ�ʱʱ����ǲ�ѯ����,ʵ��Ҫд����ʱ����
**	         uint32_t HVIN                 �����ѹ����λ10mV.��������100V������Ϊ 10000(10mV);
**           uint16_t* p_Res_HV2_To_GND    HV2�Եؾ�Ե����ָ��
**           uint16_t* p_Res_FLQ_To_GND    FLQ�Եؾ�Ե����ָ��
**
**  ����ֵ:  0                             OK  
**           1                             err 
***********************************************************/
uint8_t Res_Measure_Interface(uint8_t Timer_Counter, uint32_t HVIN, uint16_t* p_Res_HV2_To_GND, uint16_t* p_Res_FLQ_To_GND)
{
    R_Measure(Timer_Counter,HVIN);    
    *p_Res_HV2_To_GND = R_HV2_To_GND;
    *p_Res_FLQ_To_GND = R_FLQ_To_GND;
    return 0;    
}

