#define _ui_h
#include <stdint.h>
//#include <oled.h>

#include "headfile.h"

#ifndef FALSE
  #define  FALSE  0x00u                /* Boolean value FALSE. FALSE is defined always as a zero value. */
#endif
#ifndef TRUE
  #define  TRUE   0x01u                /* Boolean value TRUE. TRUE is defined always as a non zero value. */
#endif

#ifndef NULL
  #define  NULL   0x00u
#endif

typedef void (*const tIsrFunc)(void);



/* PE types definition */
  #ifndef bool 
typedef unsigned char           bool;




/******************************************
 ********UI�����������豸�ź�֮��Ķ�Ӧ��ϵ**********
 ******************************************/
typedef enum{
	UI_None			= 'Z',
	UI_Value_Plus 		= 'A',
	UI_Value_Minus 		= 'B',
	UI_Item_Plus		= 'C',
	UI_Item_Minus		= 'D',
	UI_Page_Plus		= 'E',
	UI_Page_Minus		= 'F',
	UI_Accuracy_Plus	= 'G',
	UI_Accuracy_Minus	= 'H',
	UI_Save			= 'Q',
	UI_Read		    = 'Y',
	UI_Start        = 'M'
}UI_CMD_e,*UI_CMD_Ptr_e;

/*************************
 **�����Ĳ������������������ͺ�Ȩ��**
 *************************/
typedef struct {
	enum{
		Double,
		Float,
		Int32_t,
		Uint32_t,
		Int16_t,
		Uint16_t,
		Int8_t,
		Uint8_t,
		Function,	//��ʾ��ʾͼ��ĺ���
	}Type;

	enum{
		RO,			//ֻ��
		RW,			//��д
	}Authority;
}Variable_Attribute;

/********************************************
 ************һ��Item���ر���Ԫ��*****************
 ********************************************/
typedef struct{
	signed char*		Name;				//Item�����ַ�������
	void*				DataPtr;			//���Itemָ������ݵ�ַ
	Variable_Attribute	Data_Attribute;		//���Item��Ӧ���ݵĲ���
}Item_Lib,*Item_LibPtr;

/********************************************
 ************һ��Page���ر���Ԫ��*****************
 ********************************************/
typedef struct{
	signed char*		Name;				//Page�����ַ�������
	Item_LibPtr			ItemPtr;			//���Page�ĵ�һ��Item���׵�ַ
	uint16_t			Item_Max;			//���Page��Ӧ������Item����
}Page_Lib,*Page_LibPtr;

/********************************************
 ***************���浱ǰ�Ļ״̬*****************
 ********************************************/
typedef struct{
	float 				Accuracy;			//��ǰ���ݵ�������
	bool				ShowGraphic;		        //����Ƿ���ͼ����ʾ
	int16_t				PageNum;			//��ǰ��Page�����ͣ�������
	Page_LibPtr			PagePtr;			//��ǰPageָ���Page��Ϣ�׵�ַ�����ͣ�ָ��
	int16_t				ItemNum;			//��ǰ��Item�����ͣ�������
	Item_LibPtr			ItemPtr;			//��ǰItemָ���Item��Ϣ�׵�ַ�����ͣ�ָ��
}Active_Type,*Active_TypePtr;

void UI_Init(void);
void UI_Main(void);


void UI_GetCmd(void);
void Read_Data(void);
void Save_Data(void);

extern UI_CMD_e UI_CMD;

#endif /* UI_H_ */
