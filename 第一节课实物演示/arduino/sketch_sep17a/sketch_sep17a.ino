#include <Arduino.h>


//stm32duino
//步进电机驱动 需要 3个信号线进行控制
const int ena_pin = PA4;
const int dir_pin = PA5;
const int plu_pin = PA6;

void setup()
{
  //设置引脚工作，模式
  pinMode(ena_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(plu_pin, OUTPUT);

  //使能端开启
  digitalWrite(ena_pin, LOW);
}

void driver_run(int plu, int duration_us)
{
  //判断电机旋转方向
  if (plu > 0)
  {
    digitalWrite(dir_pin, HIGH);
  }
  else
  {
    digitalWrite(dir_pin, LOW);
  }

  //输出脉冲信号
  for (int i = 0 ; i < abs(plu); i ++)
  {
    digitalWrite(plu_pin, HIGH);
    delayMicroseconds(duration_us);
    digitalWrite(plu_pin, LOW);
    delayMicroseconds(duration_us);
  }
}

int main()
{
  init();
  setup();
  
  while (1)
  {
    //旋转180°
    driver_run(6400 / 2, 50);
    delay(1000);
    //旋转-90°
    driver_run(-6400 / 4, 100);
    delay(1000);
  }
  return 0;
}



//串口烧录失败 sudo chmod 777 /dev/tty*
// sudo usermod -aG dialout 用户名
