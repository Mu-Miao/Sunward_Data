
#include <SSD1306.h>

////////OLED显示屏引脚///////////
//OLED丝印标识--------ArduinoUNO主板引脚
//
//        GND-----------------GND
//        VCC-----------------5V
//        SCL----------------- 8
//        SDA----------------- 7
//        RES----------------- 6
//        DC ----------------- 5

//OLED丝印标识--------ArduinoUNO主板引脚


#define OLED_CLK 8
#define OLED_MOSI 7
#define OLED_RESET 6
#define OLED_DC 5
SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, 0);
void setup()   {                
   oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
   oled.clear();   // clears the screen and buffer
   delay(200);    //延时等待初始化完成
}

void loop()                     
{
      static int count=0;//定义一个自加变量
    if(++count>100) count=0;//大于100清零
    oled.drawstring(15,02,"MiniBalance");//显示"MiniBalance"
    OLED_ShowNumber(30,05,count,3);//显示变量"count"
    oled.display();//刷新
    delay(50);//延时
}

/**************************************************************************
函数功能：求次方的函数
入口参数：m,n
返回  值：m的n次幂
**************************************************************************/
uint32_t oled_pow(uint8_t m,uint8_t n)
{
  uint32_t result=1;  
  while(n--)result*=m;    
  return result;
} 
/**************************************************************************
函数功能：显示变量
入口参数：x:x坐标   y:行     num：显示的变量   len ：变量的长度
返回  值：无
**************************************************************************/
void OLED_ShowNumber(uint8_t x,uint8_t y,uint32_t num,uint8_t len)
{           
  u8 t,temp;
  u8 enshow=0;               
  for(t=0;t<len;t++)
  {
    temp=(num/oled_pow(10,len-t-1))%10;
    oled.drawchar(x+6*t,y,temp+'0');
  }  
} 






