/*
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.io"
__copyright__ = "Copyright 2021 Westwood Robotics"
__date__ = "Nov. 04, 2021"

__version__ = "0.0.3"
__status__ = "Prototype" 
*/
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/spi.h" 
#include "pico/binary_info.h"

#include <nRF24L01.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// NRF24L01
#define PIN_SCK 2  
#define PIN_MOSI 3
#define PIN_MISO 4
#define PIN_CSN 1
#define PIN_CE 0
#define PIN_IRQ 5
// Others
#define PIN_STOP_IN      9  //急停输入 !!!not in use!!!
#define PIN_SSTOP_IN     8  //软急停输入
#define PIN_SSTOP_OUT    13 //软急停输出
#define PIN_STOP_LED     10  //急停显示 !!!not in use!!!
#define PIN_SSTOP_LED    6  //软急停显示
#define PIN_STATUS       7 //offline状态显示
#define loop_time        2000 // Loop time control (us)
#define FILTER_MAX_COUNT 100 // remote filter max count
#define BLINK_COUNT      3   // Warning blink count


#define TX_ADR_WIDTH 5
#define TX_PLOAD_WIDTH 6
#define ACK_PLOAD_WIDTH 6

// Which GPIO pin on is connected to the NeoPixels?
#define NeoPIN 16 

// How many NeoPixels 
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, NeoPIN, NEO_GRB + NEO_KHZ800);

unsigned char txbuf[TX_PLOAD_WIDTH];
unsigned char rxbuf[TX_PLOAD_WIDTH];  //存放数据
unsigned char ackbuf[ACK_PLOAD_WIDTH];  //存放数据
unsigned char ADDRESS1[TX_ADR_WIDTH] = {0x78,0x78,0x78,0x78,0x78};
unsigned char ADDRESS2[TX_ADR_WIDTH] = {0xC2,0xC2,0xC2,0xC2,0xC2};

// Global variables
uint8_t local_status = 0b00000000;  // LSB->MSB: STOP, SSTOP, Error
uint8_t remote_status = 0b00000000; // LSB->MSB: STOP, SSTOP
uint8_t prev_remote_status = 0b00000000;
uint8_t switch_status = 0b00000000; // LSB->MSB: STOP, SSTOP
uint8_t error = 0;
uint8_t remote_count = 0;
uint8_t warning = 0;
int     warning_timer = 0;


uint8_t get_swtich_status(uint8_t SSTOP, uint8_t ESTOP){
  // get swtich status
  uint8_t SSTOP_status = gpio_get(SSTOP);
  uint8_t ESTOP_status = gpio_get(ESTOP);
  uint8_t local = 0;
  local = (SSTOP_status<<1) | ESTOP_status;
  return local;   
}

static void remote_status_filter(){
  // Filter false signal detected due to interference
  if(remote_status != prev_remote_status){
    // difference detected
    remote_count++;
    if(remote_count > FILTER_MAX_COUNT){
      // Believe it, reset count
      remote_count = 0;
      prev_remote_status = remote_status;
    }
    else{
      // Don't believe it
      remote_status = prev_remote_status;
    }
  }
  else{
    // difference was not detected
    // reset count
    if(remote_count){
      // Signal interference just happened and finished
      Serial.print("Interference detected. Hits:  ");
      Serial.println(remote_count);
      remote_count = 0;
      warning = BLINK_COUNT; // Set to blink warning for BLINK_COUNT times
      warning_timer = millis();
    }    
  }
}


void setup() {
    //初始化spi0
    spi_init(spi0,1000*1000);
    Serial.begin(115200);

    //定义引脚
    gpio_set_function(PIN_SCK,GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI,GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO,GPIO_FUNC_SPI);

    //防噪声，拉高引脚
    gpio_pull_up(PIN_MISO);
    gpio_pull_up(PIN_MOSI);
    gpio_pull_down(PIN_SCK);

    //CSN与CE引脚
    gpio_init(PIN_CSN);
    gpio_set_dir(PIN_CSN,GPIO_OUT);
    gpio_put(PIN_CSN,1);
    gpio_init(PIN_CE);
    gpio_set_dir(PIN_CE,GPIO_OUT);
    gpio_put(PIN_CE,0);

    //IRQ引脚用于判断数据是否发送成功
    gpio_init(PIN_IRQ);
    gpio_set_dir(PIN_IRQ,GPIO_IN);

    delay(300);

    //接收端配置
    reg_write(spi0,W_REGISTER+TX_ADDR,ADDRESS1,TX_ADR_WIDTH);
    reg_write(spi0,W_REGISTER+RX_ADDR_P0,ADDRESS1,TX_ADR_WIDTH);
    
    //发送端寄存器配置
    uint8_t CONFIG2 = 0x01;  //自动应答(P0)
    write_register(W_REGISTER+EN_AA, CONFIG2);
 
//    uint8_t CONFIG3 = 0x03;  //地址宽度（5字节）
//    write_register(W_REGISTER+SETUP_AW, CONFIG3);   
      
    uint8_t CONFIG4 = 0x02;  //自动重发两次
    write_register(W_REGISTER+SETUP_RETR, CONFIG4);
    
    uint8_t CONFIG5 = 0x00;  //设置射频通道频率
    write_register(W_REGISTER+RF_CH, CONFIG5);
    
    uint8_t CONFIG6 = 0x0F;  //设置射频配置
    write_register(W_REGISTER+RF_SETUP, CONFIG6);
    
    uint8_t CONFIG7 = 0x01;  //接收地址（pipe 0）
    write_register(W_REGISTER+EN_RXADDR, CONFIG7); 
     
    uint8_t CONFIG1 = 0b01001011;  //设置芯片为接收模式+MASK_RX_DR
    write_register(W_REGISTER+NRF_CONFIG, CONFIG1);

    uint8_t CONFIG_FEATURE = 0b00000110; // DPL, Payload with ACK
    write_register(W_REGISTER+0x1D, CONFIG_FEATURE);

    uint8_t CONFIG_DYNPD = 0x01; // En. DPL on pipe 0
    write_register(W_REGISTER+0x1C, CONFIG_DYNPD);

    gpio_put(PIN_CE,1); // TODO: work on this part for stand-by and power saving

    write_register(FLUSH_TX,0xFF);
    write_register(FLUSH_RX,0xFF);

    // Buttons and LEDs
    gpio_init(PIN_STOP_IN);      
    gpio_set_dir(PIN_STOP_IN,GPIO_IN);
    gpio_pull_up(PIN_STOP_IN);

    gpio_init(PIN_SSTOP_IN);      
    gpio_set_dir(PIN_SSTOP_IN,GPIO_IN);
    gpio_pull_up(PIN_SSTOP_IN);

    gpio_init(PIN_STOP_LED);
    gpio_set_dir(PIN_STOP_LED,GPIO_OUT);
    gpio_put(PIN_STOP_LED,0);
    
    gpio_init(PIN_SSTOP_LED);
    gpio_set_dir(PIN_SSTOP_LED,GPIO_OUT);
    gpio_put(PIN_SSTOP_LED,0);

    // Signal ports        
    gpio_init(PIN_SSTOP_OUT);
    gpio_set_dir(PIN_SSTOP_OUT,GPIO_OUT);
    gpio_put(PIN_SSTOP_OUT, 1);

    gpio_init(PIN_STATUS);
    gpio_set_dir(PIN_STATUS,GPIO_OUT);
    gpio_put(PIN_STATUS,1);
    
    // INITIALIZE NeoPixel strip object (REQUIRED)
    pixels.begin(); 
    // Light up for 500ms    
    pixels.setPixelColor(0, pixels.Color(0, 150, 0, 150));
    pixels.show();
    delay(300);
    pixels.clear();
    pixels.show();
}

void loop() {

  int t = micros();
  bool ot = false;
  error = 0;
  
  // Write ACK_Payload
  ackbuf[0] = local_status;
  reg_write(spi0, W_ACK_PAYLOAD+0b000, ackbuf, 1);

  // Monitor IRQ
  uint8_t RX_DR_IRQ = gpio_get(PIN_IRQ);
  uint8_t rx_full = 0; 

  while(RX_DR_IRQ){
    // IRQ is low when RX_DR is set to 1
    reg_read(spi0, R_REGISTER+FIFO_STATUS, rxbuf, 2);
    // Check if RX full
    rx_full = rxbuf[1] & 0b00000010;
    if(rx_full){
      // RX full but no new data
      write_register(FLUSH_RX,0xFF);
    }
    // Check if RX_DR
    if(rxbuf[0] & 0b01000000){
      // RX_DR noticed
      break;
    }
    delayMicroseconds(10);
    RX_DR_IRQ = gpio_get(PIN_IRQ);
    if(micros()-t>loop_time){
      ot = true; 
      error = 1;
      break;
    }
  }
  // Out of loop
  if(!ot){
    // RX_DR pulled low or RX_DR noticed
    // No overtime
//  Serial.println("Got data.");
//  Serial.print("STATUS:  ");
//  Serial.print(rxbuf[0]);
//  Serial.print("   FIFO STATUS:  ");
//  Serial.println(rxbuf[1]);

    reg_read(spi0, R_RX_PAYLOAD, rxbuf, 2);
//  Serial.println("Read Payload:");
//  Serial.println("STATUS:   ");
//  Serial.print(rxbuf[0]);
//  Serial.print("   Payload:  ");
//  Serial.println(rxbuf[1]); // rxbuf[1] is the data we are looking for.
  
    remote_status = rxbuf[1];

    // Now we are looking for TX_DS
    uint8_t TX_DS_bit = rxbuf[0] & 0b00100000;
    while(!TX_DS_bit){
      delayMicroseconds(10);
      reg_read(spi0, 0xFF, rxbuf, 1);
      TX_DS_bit = rxbuf[0] & 0b00100000;
      if(micros()-t>loop_time){
        ot = true; 
        break;
      }
    }
  }
  // Out of loop
  // TX_DS noticed or overtime
  if(ot){
    // Overtime
    // Only take local switch status
    error = 1;
    remote_status = 0b00000000;
  }

  write_register(W_REGISTER+NRF_STATUS, rxbuf[0]);
  write_register(FLUSH_TX,0xFF);
  write_register(FLUSH_RX,0xFF);

  // We don't have ESTOP on remote so remove:
  remote_status = remote_status & 0b00000010;

  switch_status = get_swtich_status(PIN_SSTOP_IN, PIN_STOP_IN);
  remote_status_filter();
  local_status = switch_status | remote_status | (error<<2);
  //  update error display
  gpio_put(PIN_STATUS,error);
  warning_blink();
  //  update stop display
  gpio_put(PIN_STOP_LED, (local_status & 0b00000001));
  gpio_put(PIN_SSTOP_LED, (local_status & 0b00000010));  

  // take action
  if(local_status & 0b00000010){
    gpio_put(PIN_SSTOP_OUT, 1);    
  }
  else{
    gpio_put(PIN_SSTOP_OUT, 0);
  }
//  Serial.println("    ");
//  Serial.println("    ");
}
