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
