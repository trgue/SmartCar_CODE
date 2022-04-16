#include "ui.h"
#include "PID_Speed.h"
#include "Camera.h"
#include "Balance.h"
#include "tuoluoyi.h"
#include "isr.h"
#include "headfile.h"



#define Page0_Item_Max 	(sizeof(Page0_Item)/sizeof(Page0_Item[0])-1)
#define Page1_Item_Max 	(sizeof(Page1_Item)/sizeof(Page1_Item[0])-1)
#define Page2_Item_Max  (sizeof(Page2_Item)/sizeof(Page2_Item[0])-1)
#define Page3_Item_Max  (sizeof(Page3_Item)/sizeof(Page3_Item[0])-1)
#define Page4_Item_Max  (sizeof(Page4_Item)/sizeof(Page4_Item[0])-1)
#define Page5_Item_Max  (sizeof(Page5_Item)/sizeof(Page5_Item[0])-1)
#define Page6_Item_Max  (sizeof(Page6_Item)/sizeof(Page6_Item[0])-1)

float bianliang = 0;
uint8 Bluetooth_Rdat = 0;//�������ڽ��յ�����




Item_Lib const Page0_Item[] =
{



        "P",&flywheel_balance_P,Float,RW,
        "I",&flywheel_balance_I,Float,RW,
        "D",&flywheel_balance_D,Float,RW,
        "error",&error_my,Float,RW,
        "speed",&flywheel_duty,Int16_t,RW,
        "encoder",&encoder_flywheel,Int16_t,RW,










};

Item_Lib const Page5_Item[] =
{
        "FLAG",&flag_on,Int16_t,RW,
        "P_speed",&flywheel_speed_P,Float,RW,
        "I_speed",&flywheel_speed_I,Float,RW,
        "speed",&flywheel_duty,Int16_t,RW,
        "encoder",&encoder_flywheel,Int16_t,RW,
        "error",&error_my,Float,RW,



};

Item_Lib const Page4_Item[] =
{

        "pitch",&pitch,Float,RW,
//        "yaw",&yaw,Float,RW,
        "roll",&roll,Float,RW,
        "P_SD",&SD_B_P,Float,RW,
        "I_SD",&SD_B_I,Float,RW,
        "D_SD",&SD_B_D,Float,RW,
        "SD",&SD_duty,Int16_t,RW,

//                "ccqd",&t,Float,RW,
//                "zz",&zz,Float,RW,
//                "yz",&yz,Float,RW,
//                "jinhuan_flag",&jinhuan_flag,Float,RW,
//                "huan_zhuan",&huan_zhuan,Float,RW,
//                "jin",&jin,Float,RW,
//               "js1",&js1,Float,RW,


};


Item_Lib const Page3_Item[] =
{
        "Angle",&Angle,Float,RW,
        "error",&error_my,Float,RW,
        "ZERO",&Angle_Zero,Float,RW,


//        "encoder",&encoder,Float,RW,
//        "speed1",&speed1,Float,RW,
//        "speed2",&speed2,Float,RW,
//        "tui",&tui,Float,RW,
//        "da",&da,Float,RW,
////        "jian",&jiansu,Float,RW,

};

Item_Lib const Page2_Item[] =
{
              "SP",&PID_Speed_P,Float,RW,
              "SI",&PID_Speed_I,Float,RW,
              "SD",&PID_Speed_D,Float,RW,
              "PWM",&wheel_duty,Int16_t,RW,
              "EN",&encoder,Int16_t,RW,

//        "ting",&ting_flag,Float,RW,
//        "kaiqi",&kaiqi,Float,RW,
//        "fache",&fache,Float,RW,
//
//        "wan",&wan,Float,RW,
//        "jin",&jin,Float,RW,
//       "js1",&js1,Float,RW,
};


Item_Lib const Page1_Item[] =
{
        "bianliang",&bianliang,Float,RW,


};


Item_Lib const Page6_Item[] =
{

        "bianliang",&bianliang,Float,RW,


};


Page_Lib const Page[] =
{
	{"g1h", (Item_LibPtr)Page0_Item, Page0_Item_Max},
	{"huandao", (Item_LibPtr)Page1_Item, Page1_Item_Max},
	{"zhijiao", (Item_LibPtr)Page2_Item, Page2_Item_Max},
	{"sudu", (Item_LibPtr)Page3_Item, Page3_Item_Max},
    {"diangan", (Item_LibPtr)Page4_Item, Page4_Item_Max},
    {"wu", (Item_LibPtr)Page5_Item, Page5_Item_Max},
    {"sha", (Item_LibPtr)Page6_Item, Page6_Item_Max},




};

UI_CMD_e UI_CMD;

//Key_Type_e Key_Detected;

Active_Type Active;

uint16_t const Page_MAX = (sizeof(Page) / sizeof(Page[0]) - 1);

void UI_Init(void) 
{
//	Key_Init();
	//oled_init();
        lcd_init();

//	eeprom_init();
        eeprom_erase_sector(0);

	Active.ShowGraphic = FALSE;
	Active.Accuracy = 10;
	Active.PageNum = 0;
	Active.ItemNum = 0;
	Active.PagePtr = (Page_LibPtr) Page + Active.PageNum;
	Active.ItemPtr = (Active.PagePtr->ItemPtr + Active.ItemNum);
//	Key_Detected = Key_None;
	UI_CMD = UI_None;
}

uint32_t Save_Addr;
int16_t Page_Num,Item_Num;

//void Save_Data(void)
//{
//	Storage_Erase(FLASH_USER_AREA0_User_ADDRESS,FLASH_USER_AREA0_User_SIZE);
//	Save_Addr = FLASH_USER_AREA0_User_ADDRESS;
//	for(Page_Num = 0;Page_Num <= Page_MAX;Page_Num++)
//	{
//		for(Item_Num = 0;Item_Num <= Page[Page_Num].Item_Max;Item_Num++)
//		{
//			if(Save_Addr > FLASH_USER_AREA0_User_ADDRESS+FLASH_USER_AREA0_User_SIZE){return;}
//			Storage_Write(Save_Addr,(Page[Page_Num].ItemPtr+Item_Num)->DataPtr,4);
//			Save_Addr+=4;
//		}
//	}
//}

uint32_t Read_Addr;

//void Read_Data(void)
//{
//	int16_t Page_Num,Item_Num;
//	Read_Addr = FLASH_USER_AREA0_User_ADDRESS;
//	for(Page_Num = 0;Page_Num <= Page_MAX;Page_Num++)
//	{
//		for(Item_Num = 0;Item_Num <= Page[Page_Num].Item_Max;Item_Num++)
//		{
//			if(Read_Addr > FLASH_USER_AREA0_User_ADDRESS+FLASH_USER_AREA0_User_SIZE){return;}
//			Storage_Read(Read_Addr,(Page[Page_Num].ItemPtr+Item_Num)->DataPtr,4);
//			Read_Addr+=4;
//		}
//	}
//}

void Page_Plus(void) 
{

	Active.ShowGraphic = FALSE;
	Active.PageNum++;
	if (Active.PageNum > Page_MAX)
		Active.PageNum = 0;
	Active.PagePtr = (Page_LibPtr) Page + Active.PageNum;
	Active.ItemNum = 0;
	Active.ItemPtr = Active.PagePtr->ItemPtr + Active.ItemNum;
}

void Page_Minus(void) 
{

	Active.ShowGraphic = FALSE;
	Active.PageNum--;
	if (Active.PageNum < 0)
		Active.PageNum = Page_MAX;
	Active.PagePtr = (Page_LibPtr) Page + Active.PageNum;
	Active.ItemNum = 0;
	Active.ItemPtr = Active.PagePtr->ItemPtr + Active.ItemNum;
}

void Item_Plus(void) 
{

	Active.ShowGraphic = FALSE;
	Active.ItemNum++;
	if (Active.ItemNum > Active.PagePtr->Item_Max)
		Active.ItemNum = 0;
	Active.ItemPtr = Active.PagePtr->ItemPtr + Active.ItemNum;
}

void Item_Minus(void) 
{

	Active.ShowGraphic = FALSE;
	Active.ItemNum--;
	if (Active.ItemNum < 0)
		Active.ItemNum = Active.PagePtr->Item_Max;
	Active.ItemPtr = Active.PagePtr->ItemPtr + Active.ItemNum;
}

void Accuracy_Plus(void) 
{

	Active.Accuracy *= 10;
	if (Active.Accuracy > 1000)
		Active.Accuracy = 0.1;
}

void Accuracy_Minus(void) 
{

	Active.Accuracy /= 10;
	if (Active.Accuracy < 0.1)
		Active.Accuracy = 1000;
}

void Data_Plus(void) 
{

	switch (Active.ItemPtr->Data_Attribute.Type) {
	case Double:
		*(double*) Active.ItemPtr->DataPtr += (double) Active.Accuracy;
		break;
	case Float:
		*(float*) Active.ItemPtr->DataPtr += (float) Active.Accuracy;
		break;
	case Int32_t:
		*(int32_t*) Active.ItemPtr->DataPtr += (int32_t) Active.Accuracy;
		break;
	case Uint32_t:
		*(uint32_t*) Active.ItemPtr->DataPtr += (uint32_t) Active.Accuracy;
		break;
	case Int16_t:
		*(int16_t*) Active.ItemPtr->DataPtr += (int16_t) Active.Accuracy;
		break;
	case Uint16_t:
		*(uint16_t*) Active.ItemPtr->DataPtr += (uint16_t) Active.Accuracy;
		break;
	case Int8_t:
		*(int8_t*) Active.ItemPtr->DataPtr += (int8_t) Active.Accuracy;
		break;
	case Uint8_t:
		*(uint8_t*) Active.ItemPtr->DataPtr += (uint8_t) Active.Accuracy;
		break;
	case Function:
		Active.ShowGraphic = TRUE;
		break;
	default:
		break;
	}
}

void Data_Minus(void) 
{

	switch (Active.ItemPtr->Data_Attribute.Type) {
	case Double:
		*(double*) Active.ItemPtr->DataPtr -= (double) Active.Accuracy;
		break;
	case Float:
		*(float*) Active.ItemPtr->DataPtr -= (float) Active.Accuracy;
		break;
	case Int32_t:
		*(int32_t*) Active.ItemPtr->DataPtr -= (int32_t) Active.Accuracy;
		break;
	case Uint32_t:
		*(uint32_t*) Active.ItemPtr->DataPtr -= (uint32_t) Active.Accuracy;
		break;
	case Int16_t:
		*(int16_t*) Active.ItemPtr->DataPtr -= (int16_t) Active.Accuracy;
		break;
	case Uint16_t:
		*(uint16_t*) Active.ItemPtr->DataPtr -= (uint16_t) Active.Accuracy;
		break;
	case Int8_t:
		*(int8_t*) Active.ItemPtr->DataPtr -= (int8_t) Active.Accuracy;
		break;
	case Uint8_t:
		*(uint8_t*) Active.ItemPtr->DataPtr -= (uint8_t) Active.Accuracy;
		break;
	case Function:
		Active.ShowGraphic = FALSE;
		break;
	default:
		break;
	}
}

void UI_Process(void) 
{

	switch (UI_CMD) {
	case UI_Value_Plus:
		Data_Plus();
//		Save_Data();
		break;
	case UI_Value_Minus:
		Data_Minus();
//		Save_Data();
		break;
	case UI_Page_Plus:
		Page_Plus();
		break;
	case UI_Page_Minus:
		Page_Minus();
		break;
	case UI_Item_Plus:
		Item_Plus();
		break;
	case UI_Item_Minus:
		Item_Minus();
		break;
	case UI_Accuracy_Plus:
		Accuracy_Plus();
		break;
	case UI_Accuracy_Minus:
		Accuracy_Minus();
		break;
	default:
		break;
	}
}



void UI_ShowData(uint8 y,Item_LibPtr ItemPtr)
{
    lcd_showstr(10,y,ItemPtr->Name);
    switch (ItemPtr->Data_Attribute.Type) {
	case Float:
		lcd_showfloat(60, y, (float) *(float*) ItemPtr->DataPtr,5,1);
		break;
	case Int32_t:
		lcd_showint32(60, y, (int32_t) *(int32_t*) ItemPtr->DataPtr,5);
		break;
	case Uint32_t:
		lcd_showint32(60, y, (int32_t) *(int32_t*) ItemPtr->DataPtr,5);
		break;
	case Int16_t:
		lcd_showint16(60, y, (int32_t) *(int16_t*) ItemPtr->DataPtr);
		break;
	case Uint16_t:
		lcd_showint16(60, y, (int32_t) *(uint16_t*) ItemPtr->DataPtr);
		break;
	case Int8_t:
	    lcd_showuint8(60, y, (int32_t) *(uint16_t*) ItemPtr->DataPtr);


		break;
	}
}

//void UI_ShowData(uint8_t y, Item_LibPtr ItemPtr) 
//{
//
//	lcd_showstr(y, 8, ItemPtr->Name);
//	switch (ItemPtr->Data_Attribute.Type) {
//	case Double:
//		Dis_Float(y, (float) *(double*) ItemPtr->DataPtr);
//		break;
//	case Float:
//		Dis_Float(y, (float) *(float*) ItemPtr->DataPtr);
//		break;
//	case Int32_t:
//		Dis_Int(y, (int32_t) *(int32_t*) ItemPtr->DataPtr);
//		break;
//	case Uint32_t:
//		Dis_Int(y, (int32_t) *(uint32_t*) ItemPtr->DataPtr);
//		break;
//	case Int16_t:
//		Dis_Int(y, (int32_t) *(int16_t*) ItemPtr->DataPtr);
//		break;
//	case Uint16_t:
//		Dis_Int(y, (int32_t) *(uint16_t*) ItemPtr->DataPtr);
//		break;
//	case Int8_t:
//		Dis_Int(y, (int32_t) *(int8_t*) ItemPtr->DataPtr);
//		break;
//	case Uint8_t:
//		Dis_Int(y, (int32_t) *(uint8_t*) ItemPtr->DataPtr);
//		break;
//	case Function:
//		lcd_showstr(y, 100, "Fun");
//	}
//}

void UI_GetCmd(void)//�Ӵ��ڻ�ȡ����
{
//	static Key_Type_e Key_Pressed_New = Key_None;
//	static Key_Type_e Key_Pressed_Old = Key_None;
//
//	Key_Pressed_New = Keyscan();

	if (uart_query (UART_0 , &Bluetooth_Rdat) )
	{
		UI_CMD = Bluetooth_Rdat;
		Bluetooth_Rdat = 0;
	}
	else
	{
	    UI_CMD = 0;
	}
//	else
//	{
//		UI_CMD = (UI_CMD_e) Key_Pressed_New;
//		Key_Pressed_Old = Key_Pressed_New;
//	}
}

void UI_PutMsg(void) 
{

	if(UI_CMD != UI_None)
	    lcd_clear(WHITE);						//����

	lcd_showstr(0, 0, Active.PagePtr->Name);
	lcd_showfloat(60,0,Active.Accuracy,4,3);
	if (Active.ShowGraphic == FALSE) 
        {
		if (Active.ItemNum < 6) 
                {
			lcd_showstr(0, (Active.ItemNum + 1), ">");
			UI_ShowData( 1,Active.PagePtr->ItemPtr);
			if (Active.PagePtr->Item_Max<1) 
                        {return;}
			UI_ShowData( 2,Active.PagePtr->ItemPtr + 1);
			if (Active.PagePtr->Item_Max<2) 
                        {return;}
			UI_ShowData(3,Active.PagePtr->ItemPtr + 2);
                        if (Active.PagePtr->Item_Max<3) 
                        {return;}
			UI_ShowData( 4,Active.PagePtr->ItemPtr + 3);
                        if (Active.PagePtr->Item_Max<4) 
                        {return;}
			UI_ShowData( 5,Active.PagePtr->ItemPtr + 4);
                        if (Active.PagePtr->Item_Max<5) 
                        {return;}
			UI_ShowData( 6,Active.PagePtr->ItemPtr + 5);
                        
		} 
                else 
                {
			lcd_showstr(3,1, ">");
			UI_ShowData( 1,Active.ItemPtr - 5);
			UI_ShowData( 2,Active.ItemPtr - 4);
			UI_ShowData( 3,Active.ItemPtr - 3);
                        UI_ShowData( 4,Active.ItemPtr - 2);
                        UI_ShowData( 5,Active.ItemPtr - 1);
                        UI_ShowData( 6,Active.ItemPtr);
		}
	} 
        else 
        {
		(*(tIsrFunc) Active.ItemPtr->DataPtr)();
	}
}

//void UI_SelectData(void) 
//{
//
//	if(UI_CMD == UI_Save)
//	{
//		Save_Data();
//	}
//
//	if(UI_CMD== UI_Read)
//	{
//		Read_Data();
//	}
//}

void UI_Main(void) 
{

	UI_GetCmd();
	UI_Process();
//	UI_SelectData();
	UI_PutMsg();
}
