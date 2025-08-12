#include "HW_key.h"
#include "ti_msp_dl_config.h"
#include "clock.h"
uint8_t Key_row[1]={0xff};		//keyborad input
/***
 *函数名：KEY_ROW_SCAN
 *功  能：按键行扫描
 *返回值：1~4，对应1~4行按键位置
 */

uint8_t HAL_GPIO_ReadPin(GPIO_Regs *gpio, uint32_t pins)
{
    if(DL_GPIO_readPins(gpio,pins)){return 1;}
    else return 0;
}

char KEY_ROW_SCAN(void)
{   
    //读出行扫描状态
    Key_row[0] = HAL_GPIO_ReadPin(GPIOA,DL_GPIO_PIN_14)<<3;
    Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(GPIOB,DL_GPIO_PIN_10)<<2);
    Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(GPIOB,DL_GPIO_PIN_5)<<1);
    Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(GPIOB,DL_GPIO_PIN_11));
    
    if(Key_row[0] != 0x0f)         //行扫描有变化，判断该列有按键按下
    {
      mspm0_delay_ms(10);                    //消抖
      if(Key_row[0] != 0x0f)
        {   
                //printf("Key_Row_DATA = 0x%x\r\n",Key_row[0]);
                switch(Key_row[0])
                {
                    case 0x07:         //0111 判断为该列第1行的按键按下
                        return 1;
                    case 0x0b:         //1011 判断为该列第2行的按键按下
                        return 2;
                    case 0x0d:         //1101 判断为该列第3行的按键按下
                        return 3;
                    case 0x0e:         //1110 判断为该列第4行的按键按下
                        return 4;
                    default :
                        return 0;
                }
        }
        else return 0;
    }
    else return 0;
}

/***
 *函数名：KEY_SCAN
 *功  能：4*4按键扫描
 *返回值：0~16，对应16个按键
 */

char KEY_SCAN(void)
{    
    char Key_Num=0;       //1-16对应的按键数
    char key_row_num=0;        //行扫描结果记录
    
	while(!key_row_num){
    KEY_CLO0_OUT_LOW;        
    if( (key_row_num=KEY_ROW_SCAN()) != 0 )
    {
        while(KEY_ROW_SCAN() != 0);  //消抖
        Key_Num = 0 + key_row_num;
        //printf("Key_Clo_1\r\n");
    }
    KEY_CLO0_OUT_HIGH;
	if(key_row_num!=0) break;
    
    KEY_CLO1_OUT_LOW;        
    if( (key_row_num=KEY_ROW_SCAN()) != 0 )
    { 
        while(KEY_ROW_SCAN() != 0);
        Key_Num = 4 + key_row_num;
        //printf("Key_Clo_2\r\n");
    }
    KEY_CLO1_OUT_HIGH;
    if(key_row_num!=0) break;
    KEY_CLO2_OUT_LOW;    
    if( (key_row_num=KEY_ROW_SCAN()) != 0 )
    { 
        while(KEY_ROW_SCAN() != 0);
    Key_Num = 8 + key_row_num;
	
        //printf("Key_Clo_3\r\n");
    }
    KEY_CLO2_OUT_HIGH;
    if(key_row_num!=0) break;
    KEY_CLO3_OUT_LOW;    
    if( (key_row_num=KEY_ROW_SCAN()) != 0 )
    {
//        Key_row[0] = HAL_GPIO_ReadPin(GPIOE,KEY_col0_Pin)<<3;
//        Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(GPIOE,KEY_col1_Pin)<<2);
//        Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(GPIOE,KEY_col2_Pin)<<1);
//        Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(GPIOE,KEY_col3_Pin));
//        printf("Key_Clo4_DATA = 0x%x\r\n",Key_row[0]);
        while(KEY_ROW_SCAN() != 0);
        Key_Num = 12 + key_row_num;
        //printf("Key_Clo_4\r\n");
    }
    KEY_CLO3_OUT_HIGH;
}
    return Key_Num;
}
/***
 *函数名：KEY_SCAN
 *功  能：4*4按键扫描
 *返回值：0~16，对应16个按键
 */
