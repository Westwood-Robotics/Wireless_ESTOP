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

#define TX_ADR_WIDTH 5
#define TX_PLOAD_WIDTH 6

unsigned char txbuf[TX_PLOAD_WIDTH];
unsigned char rxbuf[TX_PLOAD_WIDTH]; 

unsigned char ADDRESS1[TX_ADR_WIDTH] = {0x78,0x78,0x78,0x78,0x78};
unsigned char ADDRESS2[TX_ADR_WIDTH] = {0xC2,0xC2,0xC2,0xC2,0xC2};

// Global variables
uint8_t local_status = 0b00000000;  // LSB->MSB: STOP, SSTOP
uint8_t remote_status = 0b00000000; // LSB->MSB: STOP, SSTOP
uint8_t switch_status = 0b00000000; // LSB->MSB: STOP, SSTOP
uint8_t error = 0;
long global_time = 0;

//写入数据
int reg_write(spi_inst_t* spi,
              const uint8_t reg,
              uint8_t *buf,
              const uint8_t nbytes) {  
           
    int num_bytes_write = 0;
    uint8_t msg[nbytes + 1];

    if(nbytes < 1) {
        return 0;
   }

    msg[0] = reg;
    for(int i = 0; i < nbytes; i++) {
      msg[i + 1] = buf[i];
    }
    gpio_put(PIN_CSN,0);
    num_bytes_write = spi_write_blocking(spi, msg, (nbytes + 1));
    gpio_put(PIN_CSN,1);

    return num_bytes_write;
}


//读取数据
int reg_read( spi_inst_t* spi,
              const uint8_t reg,
              uint8_t *buf,
              const uint8_t nbytes) {

    int num_bytes_read = 0;

    if (nbytes < 1) {
        return 0;
    }
    
    gpio_put(PIN_CSN,0);
    num_bytes_read = spi_read_blocking(spi, reg, buf, nbytes);
    gpio_put(PIN_CSN,1);
    return num_bytes_read;
} 


//写入寄存器（用于寄存器设置）
static void write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;
    gpio_put(PIN_CSN,0);
    spi_write_blocking(spi0, buf, 2);
    gpio_put(PIN_CSN,1);
}


//读取寄存器（用于寄存器设置）
static void read_register(uint8_t reg, uint8_t *buf, uint16_t len) {
    gpio_put(PIN_CSN,0);
    spi_read_blocking(spi0, reg, buf, len);
    gpio_put(PIN_CSN,1);
}

uint8_t get_swtich_status(uint8_t SSTOP, uint8_t ESTOP){
  // get swtich status
  uint8_t SSTOP_status = gpio_get(SSTOP);
  uint8_t ESTOP_status = gpio_get(ESTOP);
  uint8_t local = 0;
  local = (SSTOP_status<<1) | ESTOP_status;
  return local;   
}


void setup() {
    //初始化spi0
    spi_init(spi0,1000*1000);
    Serial.begin(9600);
    
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

    delay(1000);

    //发送端配置
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
     
    uint8_t CONFIG1 = 0b00101010;  //设置芯片为发送模式+MASK_TX_DS
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
    gpio_put(PIN_SSTOP_OUT,0);

    gpio_init(PIN_STATUS);
    gpio_set_dir(PIN_STATUS,GPIO_OUT);
    gpio_put(PIN_STATUS,0);
    
    gpio_put(PIN_STATUS,1);
    delay(500);
    global_time = micros();
}

void loop() {  
  int t = micros();
  bool ot = false;  
  switch_status = get_swtich_status(PIN_SSTOP_IN, PIN_STOP_IN);
  txbuf[0] = switch_status;
  // Check if TX full
  reg_read(spi0, R_REGISTER+FIFO_STATUS, rxbuf, 2);
  if(rxbuf[0]&0x01){
    // TX full
    write_register(FLUSH_TX,0xFF);
  }
  // Check if RX full
  if(rxbuf[1] & 0b00000010){
    // RX full
    write_register(FLUSH_RX,0xFF);
  }
  
  reg_write(spi0, W_TX_PAYLOAD, txbuf, 1);  //发送信号

  // Monitor IRQ
  // uint8_t TX_DS_IRQ = gpio_get(PIN_IRQ);
  while(true){
    reg_read(spi0, R_REGISTER+FIFO_STATUS, rxbuf, 2);
//    Serial.print("STATUS:  ");
//    Serial.print(rxbuf[0]);
//    Serial.print("   FIFO STATUS:  ");
//    Serial.println(rxbuf[1]);
//    // Check if RX_DR
//    if(rxbuf[0] & 0b01000000){
//      // RX_DR noticed
////      Serial.println("RX_DR noticed but no TX_DS.");
//    }
    // Check if RX_DR
    if(rxbuf[0] & 0b00100000){
      // TX_DS noticed
      Serial.println("TX_DS.");
      break;
    }     
    // Check if RX full
    if(rxbuf[1] & 0b00000010){
      // RX full but no new data
      write_register(FLUSH_RX,0xFF);
    }
    // Check if TX Empty
    if(rxbuf[1] & 0b00010000){
      // TX Empty
      reg_write(spi0, W_TX_PAYLOAD, txbuf, 1);  //发送信号
    }
    delayMicroseconds(100);
    // TX_DS_IRQ = gpio_get(PIN_IRQ);
    if(micros()-t>loop_time){
      ot = true;       
      break;
    }
  }
  // Out of loop
  if(ot){
    // Overtime
    if(micros()-global_time > 500000){
      // It's been overtime for more than 500ms(500,000us)
      Serial.println("Overtime.\n");
      error = 1; 
    }
  }
  else{
    // TX_DS pulled low or noticed
    Serial.println("TX_DS low or bit noticed.");
    // Read ACK payload
    reg_read(spi0, R_RX_PAYLOAD, rxbuf, 2);
    remote_status = rxbuf[1];
    error = remote_status >> 2;
    global_time = micros();
  }
  
  write_register(W_REGISTER+NRF_STATUS, rxbuf[0]);
  write_register(FLUSH_TX,0xFF);
  write_register(FLUSH_RX,0xFF);  
  
  //  update error display
  gpio_put(PIN_STATUS, error);
  Serial.println(error);
  //  update stop display
  gpio_put(PIN_STOP_LED, (remote_status & 0b00000001));
  gpio_put(PIN_SSTOP_LED, (remote_status & 0b00000010));  


  // take action
//
  Serial.println("End of loop.\n");
  Serial.println("    ");
  Serial.println("    ");
  
}
