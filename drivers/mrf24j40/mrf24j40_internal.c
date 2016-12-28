/**
 * @ingroup     drivers_mrf24j40
 * @{
 *
 * @file
 * @brief       Implementation of driver internal functions
 *
 * @author      Koen Zandberg <koen@bergzand.net>
 *
 * @}
 */
#include "periph/spi.h"
#include "periph/gpio.h"
#include "xtimer.h"
#include "mrf24j40_internal.h"
#include "mrf24j40_registers.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

void mrf24j40_init(mrf24j40_t *dev)
{
    uint8_t softrst;
    uint8_t order;
    mrf24j40_hardware_reset(dev);
    /* do a soft reset */
    mrf24j40_reg_write_short(dev, MRF24J40_REG_SOFTRST, 0x7); // from manual
    do {
        softrst = mrf24j40_reg_read_short(dev, MRF24J40_REG_SOFTRST);
    } while ((softrst & 0x07) != 0);        /* wait until soft-reset has finished */

    /* Check if MRF24J40 is available */
    order = mrf24j40_reg_read_short(dev, MRF24J40_REG_ORDER);
    if (order != 0xFF) {
    }

    /* flush RX FIFO */
    mrf24j40_reg_write_short(dev, MRF24J40_REG_RXFLUSH, MRF24J40_RXFLUSH_RXFLUSH);

    /* Here starts init-process as described on MRF24J40 Manual Chap. 3.2 */
    mrf24j40_reg_write_short(dev, MRF24J40_REG_PACON2, 	( MRF24J40_PACON2_TXONTS2 |
                                                          MRF24J40_PACON2_TXONTS1 |
                                                          MRF24J40_PACON2_FIFOEN) );
    mrf24j40_reg_write_short(dev, MRF24J40_REG_TXSTBL, ( MRF24J40_TXSTBL_RFSTBL3 |
                                                         MRF24J40_TXSTBL_RFSTBL0 |
                                                         MRF24J40_TXSTBL_MSIFS2  |
                                                         MRF24J40_TXSTBL_MSIFS0  ));
    mrf24j40_reg_write_long(dev, MRF24J40_REG_RFCON1, MRF24J40_RFCON1_VCOOPT1);
    mrf24j40_reg_write_long(dev, MRF24J40_REG_RFCON2, MRF24J40_RFCON2_PLLEN);
    mrf24j40_reg_write_long(dev, MRF24J40_REG_RFCON6, ( MRF24J40_RFCON6_TXFIL |
                                                        MRF24J40_RFCON6_20MRECVR ));
    mrf24j40_reg_write_long(dev, MRF24J40_REG_RFCON7, MRF24J40_RFCON7_SLPCLKSEL1 );
    mrf24j40_reg_write_long(dev, MRF24J40_REG_RFCON8, MRF24J40_RFCON8_RFVCO );
    mrf24j40_reg_write_long(dev, MRF24J40_REG_SLPCON1, ( MRF24J40_SLPCON1_CLKOUTEN |
                                                         MRF24J40_SLPCON1_SLPCLKDIV0 ));
    mrf24j40_reg_write_short(dev, MRF24J40_REG_BBREG2, MRF25J40_BBREG2_CCAMODE1 );
    mrf24j40_reg_write_short(dev, MRF24J40_REG_CCAEDTH, 0x60);
    mrf24j40_reg_write_short(dev, MRF24J40_REG_BBREG6, MRF24J40_BBREG6_RSSIMODE2 );


    /* set interrupt pin polarity */
    mrf24j40_reg_write_long(dev, MRF24J40_REG_SLPCON0, MRF24J40_SLPCON0_INTEDGE );        /* IRQ-Pin -> rising edge */
    /* set default channel */
    mrf24j40_reg_write_long(dev, MRF24J40_REG_RFCON0, 0xf3);        /* Default channel = 26 */
    /* set default TX power - Reset default = Max -> nothing to do */
    /* reset RF state machine */
    mrf24j40_reset_state_machine(dev);

    /* mrf24j40_set_interrupts */
    mrf24j40_reg_write_short(dev, MRF24J40_REG_INTCON, ~( MRF24J40_INTCON_RXIE | MRF24J40_INTCON_TXNIE ));
    //Wait until the RFSTATE machine indicates RX state
}

void mrf24j40_reg_write_short(mrf24j40_t *dev, const uint8_t addr, const uint8_t value)
{
    spi_acquire(dev->params.spi);
    gpio_clear(dev->params.cs_pin);
    spi_transfer_reg(dev->params.spi, MRF24J40_SHORT_ADDR_TRANS | (addr << MRF24J40_ADDR_OFFSET) | MRF24J40_ACCESS_WRITE, value, 0);
    gpio_set(dev->params.cs_pin);
    spi_release(dev->params.spi);
}

uint8_t mrf24j40_reg_read_short(mrf24j40_t *dev, const uint8_t addr)
{
    char value;

    spi_acquire(dev->params.spi);
    gpio_clear(dev->params.cs_pin);
    spi_transfer_reg(dev->params.spi, MRF24J40_SHORT_ADDR_TRANS | (addr << MRF24J40_ADDR_OFFSET) | MRF24J40_ACCESS_READ, 0, &value);
    gpio_set(dev->params.cs_pin);
    spi_release(dev->params.spi);

    return (uint8_t)value;
}


void mrf24j40_reg_write_long(mrf24j40_t *dev, const uint16_t addr, const uint8_t value)
{
    uint8_t reg1, reg2;

    reg1 = MRF24J40_LONG_ADDR_TRANS | (addr >> 3);
    reg2 = (addr << 5) | MRF24J40_ACCESS_WRITE_LNG;
    spi_acquire(dev->params.spi);
    gpio_clear(dev->params.cs_pin);
    spi_transfer_byte(dev->params.spi, reg1, 0);
    spi_transfer_byte(dev->params.spi, reg2, 0);
    spi_transfer_byte(dev->params.spi, value, 0);
    gpio_set(dev->params.cs_pin);
    spi_release(dev->params.spi);
}


uint8_t mrf24j40_reg_read_long(mrf24j40_t *dev, const uint16_t addr)
{
    uint8_t reg1, reg2;

    reg1 = MRF24J40_LONG_ADDR_TRANS | (addr >> 3);
    reg2 = (addr << 5) | MRF24J40_ACCESS_READ;
    char value;
    spi_acquire(dev->params.spi);
    gpio_clear(dev->params.cs_pin);
    spi_transfer_byte(dev->params.spi, reg1, 0);
    spi_transfer_byte(dev->params.spi, reg2, &value);
    spi_transfer_byte(dev->params.spi, 0, &value);
    gpio_set(dev->params.cs_pin);
    spi_release(dev->params.spi);

    return (uint8_t)value;
}

void mrf24j40_tx_normal_fifo_read(mrf24j40_t *dev, const uint16_t offset, uint8_t *data, const size_t len)
{
    uint8_t reg1, reg2;

    reg1 = MRF24J40_LONG_ADDR_TRANS | (offset >> 3);
    reg2 = (offset << 5) | MRF24J40_ACCESS_READ;
    spi_acquire(dev->params.spi);
    gpio_clear(dev->params.cs_pin);
    spi_transfer_byte(dev->params.spi, reg1, NULL);
    spi_transfer_byte(dev->params.spi, reg2, (char *)data);
    spi_transfer_bytes(dev->params.spi, NULL, (char *)data, len);
    gpio_set(dev->params.cs_pin);
    spi_release(dev->params.spi);
}


void mrf24j40_tx_normal_fifo_write(mrf24j40_t *dev,
                                   const uint16_t offset,
                                   const uint8_t *data,
                                   const size_t len)
{
    uint16_t addr;
    uint8_t reg1;
    uint8_t reg2;

    addr = offset;

    reg1 = MRF24J40_LONG_ADDR_TRANS | (addr >> 3);
    reg2 = (addr << 5) | MRF24J40_ACCESS_WRITE_LNG;

    spi_acquire(dev->params.spi);
    gpio_clear(dev->params.cs_pin);
    spi_transfer_byte(dev->params.spi, reg1, 0);
    spi_transfer_byte(dev->params.spi, reg2, 0);
    spi_transfer_bytes(dev->params.spi, (char *)data, NULL, len);
    gpio_set(dev->params.cs_pin);
    spi_release(dev->params.spi);
}

void mrf24j40_rx_fifo_read(mrf24j40_t *dev, const uint16_t offset, uint8_t *data, const size_t len)
{
    uint16_t rx_addr;

    rx_addr = MRF24J40_RX_FIFO + offset;

    uint8_t reg1, reg2;
    reg1 = MRF24J40_LONG_ADDR_TRANS | (rx_addr >> 3);
    reg2 = (rx_addr << 5) | MRF24J40_ACCESS_READ;
    spi_acquire(dev->params.spi);
    gpio_clear(dev->params.cs_pin);
    spi_transfer_byte(dev->params.spi, reg1, NULL);
    spi_transfer_byte(dev->params.spi, reg2, (char *)data);
    spi_transfer_bytes(dev->params.spi, NULL, (char *)data, len);
    gpio_set(dev->params.cs_pin);
    spi_release(dev->params.spi);
}

void mrf24j40_rx_fifo_write(mrf24j40_t *dev, const uint16_t offset, const uint8_t *data, const size_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++) {
        mrf24j40_reg_write_long(dev, i, data[i]);
    }
}


void mrf24j40_assert_awake(mrf24j40_t *dev)
{
    if (dev->state == MRF24J40_PSEUDO_STATE_SLEEP) {
        DEBUG("[mrf24j40] Waking up\n");

        /* set uController pin to high */
        gpio_set(dev->params.sleep_pin);

        /* reset state machine */
        mrf24j40_reg_write_short(dev, MRF24J40_REG_RFCTL, MRF24J40_RFCTL_RFRST);
        mrf24j40_reg_write_short(dev, MRF24J40_REG_RFCTL, 0x00);

        /* After wake-up, delay at least 2 ms to allow 20 MHz main
         * oscillator time to stabilize before transmitting or receiving.
         */
        xtimer_usleep(MRF24J40_WAKEUP_DELAY);
    }
}

void mrf24j40_update_int_status(mrf24j40_t *dev)
{
    uint8_t instat = 0;
    uint8_t newpending = 0;
    instat = mrf24j40_reg_read_short(dev, MRF24J40_REG_INTSTAT);
    /* check if TX done */
    if(instat & MRF24J40_INTSTAT_TXNIF){
        newpending |= MRF24J40_TASK_TX_DONE |MRF24J40_TASK_TX_READY;
    }
    if(instat & MRF24J40_INTSTAT_RXIF){
        newpending |= MRF24J40_TASK_RX_READY;
    }
    /* check if RX pending */
    dev->pending |= newpending;
}

void mrf24j40_reset_tasks(mrf24j40_t *dev)
{
    dev->pending = MRF24J40_TASK_TX_DONE;
}

void mrf24j40_update_tasks(mrf24j40_t *dev)
{
    if(dev->irq_flag){
        dev->irq_flag = 0;
        mrf24j40_update_int_status(dev);
    }
}


void mrf24j40_hardware_reset(mrf24j40_t *dev)
{
    /* wake up from sleep in case radio is sleeping */
    mrf24j40_assert_awake(dev);

    /* trigger hardware reset */
    gpio_clear(dev->params.reset_pin);
    xtimer_usleep(MRF24J40_RESET_PULSE_WIDTH);  /* Datasheet - Not specified */
    gpio_set(dev->params.reset_pin);
    xtimer_usleep(MRF24J40_RESET_DELAY);        /* Datasheet - MRF24J40 ~2ms */
}

/* read TX Normal FIFO
 */
void mrf24j40_print_tx_norm_buf(mrf24j40_t *dev)
{
    uint8_t k; /* Loop counter for Long Address Registers */

    for (k = 0x0; k <= 0x07f; k++) {
//		lng_sendbuf[k] = mrf24j40_reg_read_long(&mrf24j40_devs[0], k);
        printf("mrf24j40_print_tx_norm_buf : transmitbuf[%x]= 0x%x = %c\n", (int)k, (int)mrf24j40_reg_read_long(dev, k), (int)mrf24j40_reg_read_long(dev, k));
    }
}

/* read RX FIFO */
void mrf24j40_print_rx_buf(mrf24j40_t *dev)
{
    uint16_t k; /* Loop-Counter for Long-Address-Registers */

    for (k = 0x300; k <= 0x37f; k++) {
        printf("mrf24j40_print_rx_buf : receive-buf[%d][0x%x] = %d / 0x%x = %c\n", (int)(k - 0x300), (int)(k - 0x300), (int)mrf24j40_reg_read_long(dev, k), (int)mrf24j40_reg_read_long(dev, k), (int)mrf24j40_reg_read_long(dev, k));
    }
}

