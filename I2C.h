void i2c_init() {
    TWSR = 0x00;
    TWBR = ((F_CPU / 400000L) - 16) / 2;
    TWCR = (1 << TWEN);
}

// Start I2C communication
void i2c_start() {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

// Stop I2C communication
void i2c_stop() {
    TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
    while (TWCR & (1 << TWSTO));
}

// Write data to I2C
void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}