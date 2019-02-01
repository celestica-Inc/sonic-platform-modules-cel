/*
 * switchboard.c - Driver for Phalanx Switch FPGA/CPLD.
 *
 * Author: Pradchaya Phucharoen
 *
 * Copyright (C) 2018 Celestica Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *   /
 *   \--sys
 *       \--devices
 *            \--platform
 *                \--AS23128h.switchboard
 *                    |--FPGA
 *                    |--CPLD[1..4]
 *                    |--FAN_CPLD
 *                    \--SFF
 *                        \--QSFP[1..128]
 *
 */

#ifndef TEST_MODE
#define MOD_VERSION "0.3.0"
#else
#define MOD_VERSION "TEST"
#endif

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/io.h>
#include <linux/dmi.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <uapi/linux/stat.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/jiffies.h>

static int  majorNumber;

#define CLASS_NAME "phalanx_fpga"
#define DRIVER_NAME "AS23128h.switchboard"
#define FPGA_PCI_NAME "phalanx_fpga_pci"
#define DEVICE_NAME "fwupgrade"

static bool allow_unsafe_i2c_access;

static int smbus_access(struct i2c_adapter *adapter, u16 addr,
                        unsigned short flags, char rw, u8 cmd,
                        int size, union i2c_smbus_data *data);

static int fpga_i2c_access(struct i2c_adapter *adapter, u16 addr,
                           unsigned short flags, char rw, u8 cmd,
                           int size, union i2c_smbus_data *data);

static int i2c_core_init(unsigned int master_bus, unsigned int freq_div,void __iomem *pci_bar);
static void i2c_core_deinit(unsigned int master_bus, void __iomem *pci_bar);
static int i2c_xcvr_access(u8 register_address, unsigned int portid, u8 *data, char rw);

static int fpgafw_init(void);
static void fpgafw_exit(void);

/*
========================================
FPGA PCIe BAR 0 Registers
========================================
Misc Control    0x00000000 – 0x000000FF
I2C_CH1         0x00000800 - 0x0000081C
I2C_CH2         0x00000820 - 0x0000083C
I2C_CH3         0x00000840 - 0x0000085C
I2C_CH4         0x00000860 - 0x0000087C
I2C_CH5         0x00000880 - 0x0000089C
I2C_CH6         0x000008A0 - 0x000008BC
I2C_CH7         0x000008C0 - 0x000008DC
I2C_CH8         0x000008E0 - 0x000008FC
I2C_CH9         0x00000900 - 0x0000091C
I2C_CH10        0x00000920 - 0x0000093C
I2C_CH11        0x00000940 - 0x0000095C
I2C_CH12        0x00000960 - 0x0000097C
I2C_CH13        0x00000980 - 0x0000099C
I2C_CH14        0x000009A0 - 0x000009BC
SPI Master      0x00000A00 - 0x00000BFC
PORT XCVR       0x00004000 - 0x00004FFF
*/

/* MISC       */
#define FPGA_VERSION            0x0000
#define FPGA_VERSION_MJ_MSK     0xff00
#define FPGA_VERSION_MN_MSK     0x00ff
#define FPGA_SCRATCH            0x0004
#define FPGA_BROAD_TYPE         0x0008
#define BMC_I2C_SCRATCH         0x0020
#define FPGA_SLAVE_CPLD_REST    0x0100
#define FPGA_SWITCH_RESET_CTRL  0x0104
#define FPAG_PRH_RESER_CTRL     0x0108
#define FPGA_INT_STATUS         0x0200
#define FPGA_INT_SRC_STATUS     0x0204
#define FPGA_INT_FLAG           0x0208
#define FPGA_INT_MASK           0x020c
#define FPGA_MISC_CTRL          0x0300
#define FPGA_MISC_STATUS        0x0304

/* I2C_MASTER BASE ADDR */
#define I2C_MASTER_FREQ_L           0x0800
#define I2C_MASTER_FREQ_H           0x0804
#define I2C_MASTER_CTRL             0x0808
#define I2C_MASTER_DATA             0x080c
#define I2C_MASTER_CMD              0x0810 /* Write-Only Register */
#define I2C_MASTER_STATUS           0x0810 /* Read-Only  Register */
#define I2C_MASTER_CH_1             1
#define I2C_MASTER_CH_2             2
#define I2C_MASTER_CH_3             3
#define I2C_MASTER_CH_4             4
#define I2C_MASTER_CH_5             5
#define I2C_MASTER_CH_6             6
#define I2C_MASTER_CH_7             7
#define I2C_MASTER_CH_8             8
#define I2C_MASTER_CH_9             9
#define I2C_MASTER_CH_10            10
#define I2C_MASTER_CH_11            11
#define I2C_MASTER_CH_12            12
#define I2C_MASTER_CH_13            13
#define I2C_MASTER_CH_14            14

#define I2C_MASTER_CH_TOTAL I2C_MASTER_CH_14

/* SPI_MASTER */
#define SPI_MASTER_WR_EN            0x1200 /* one bit */
#define SPI_MASTER_WR_DATA          0x1204 /* 32 bits */
#define SPI_MASTER_CHK_ID           0x1208 /* one bit */
#define SPI_MASTER_VERIFY           0x120c /* one bit */
#define SPI_MASTER_STATUS           0x1210 /* 15 bits */
#define SPI_MASTER_MODULE_RST       0x1214 /* one bit */

/* FPGA FRONT PANEL PORT MGMT */
#define SFF_PORT_CTRL_BASE          0x4000
#define SFF_PORT_STATUS_BASE        0x4004
#define SFF_PORT_INT_STATUS_BASE    0x4008
#define SFF_PORT_INT_MASK_BASE      0x400c

#define PORT_XCVR_REGISTER_SIZE     0x1000

/* PORT CTRL REGISTER
[31:7]  RSVD
[6]     RSVD
[5]     MODSEL  5
[4]     RST     4
[3:1]   RSVD
[0]     TXDIS   0
*/
#define CTRL_MODSEL   5
#define CTRL_RST      4
#define CTRL_TXDIS    0

/* PORT STATUS REGISTER
[31:6]  RSVD
[5]     IRQ         5
[4]     PRESENT     4
[3]     RSVD
[2]     TXFAULT     2
[1]     RXLOS       1
[0]     MODABS      0
*/
#define STAT_IRQ         5
#define STAT_PRESENT     4
#define STAT_TXFAULT     2
#define STAT_RXLOS       1
#define STAT_MODABS      0

/* PORT INTRPT REGISTER
[31:6]  RSVD
[5]     INT_N       5
[4]     PRESENT     4
[3]     RSVD
[2]     RSVD
[1]     RXLOS       1
[0]     MODABS      0
*/
#define INTR_INT_N      5
#define INTR_PRESENT    4
#define INTR_TXFAULT    2
#define INTR_RXLOS      1
#define INTR_MODABS     0

/* PORT INT MASK REGISTER
[31:6]  RSVD
[5]     INT_N       5
[4]     PRESENT     4
[3]     RSVD
[2]     RSVD
[1]     RXLOS_INT   1
[0]     MODABS      0
*/
#define MASK_INT_N      5
#define MASK_PRESENT    4
#define MASK_TXFAULT    2
#define MASK_RXLOS      1
#define MASK_MODABS     0


/**
 * Switchboard CPLD XCVR registers
 */

/* PORT SEL REGISTER
[7:5]   RSVD
[4:0]   ID
*/
#define I2C_XCVR_SEL    0x10
#define I2C_SEL_ID      0

/* PORT CTRL REGISTER
[7:5]   RSVD
[4]     RST
[3:1]   RSVD
[0]     TXDIS/MODSEL
*/
#define I2C_XCVR_CTRL      0x11
#define I2C_CTRL_RST       4
#define I2C_CTRL_MODSEL    0
#define I2C_CTRL_TXDIS     0

/* PORT STATUS REGISTER
[7:5]   RSVD
[4]     PRESENT/ABS
[3:2]   RSVD
[1]     TXFAULT
[0]     RXLOS/INT_N
*/
#define I2C_XCVR_STAT        0x12
#define I2C_STAT_PRESENT     4
#define I2C_STAT_MODABS      4
#define I2C_STAT_TXFAULT     1
#define I2C_STAT_INT_N       0
#define I2C_STAT_RXLOS       0

/* PORT INTRPT REGISTER
[7:5]   RSVD
[4]     PRESENT/ABS
[3:2]   RSVD
[1]     TXFAULT
[0]     RXLOS/INT_N
*/
#define I2C_XCVR_INRT        0x13
#define I2C_INTR_PRESENT     4
#define I2C_INTR_MODABS      4
#define I2C_INTR_TXFAULT     1
#define I2C_INTR_INT_N       0
#define I2C_INTR_RXLOS       0

/* PORT INTR MASK REGISTER
[31:6]  RSVD
[5]     INT_N       5
[4]     PRESENT     4
[3]     RSVD
[2]     RSVD
[1]     RXLOS_INT   1
[0]     MODABS      0
*/
#define I2C_XCVR_MASK        0x14
#define I2C_MASK_PRESENT     4
#define I2C_MASK_MODABS      4
#define I2C_MASK_TXFAULT     1
#define I2C_MASK_INT_N       0
#define I2C_MASK_RXLOS       0


/* I2C master clock speed */
// NOTE: Only I2C clock in normal mode is support here.
enum {
    I2C_DIV_100K = 0x80,
};

/* I2C Master control register */
enum {
    I2C_CTRL_IEN = 6,
    I2C_CTRL_EN
};

/* I2C Master command register */
enum {
    I2C_CMD_IACK = 0,
    I2C_CMD_ACK = 3,
    I2C_CMD_WR,
    I2C_CMD_RD,
    I2C_CMD_STO,
    I2C_CMD_STA,
};

/* I2C Master status register */
enum {
    I2C_STAT_IF = 0,
    I2C_STAT_TIP,
    I2C_STAT_AL = 5,
    I2C_STAT_BUSY,
    I2C_STAT_RxACK,
};

/**
 *
 * The function is i2c algorithm implement to allow master access to
 * correct endpoint devices trough the PCA9548 switch devices.
 *
 *  FPGA I2C Master [mutex resource]
 *              |
 *              |
 *    ---------------------------
 *    |        PCA9548(s)       |
 *    ---1--2--3--4--5--6--7--8--
 *       |  |  |  |  |  |  |  |
 *   EEPROM      ...          EEPROM
 *
 */

#define VIRTUAL_I2C_SFP_PORT             0
#define VIRTUAL_I2C_QSFP_PORT            128

#define SFF_PORT_TOTAL    VIRTUAL_I2C_QSFP_PORT + VIRTUAL_I2C_SFP_PORT

#define VIRTUAL_I2C_BUS_OFFSET  10
#define BB_CPLD_SLAVE_ADDR      0x0d
#define FAN_CPLD_SLAVE_ADDR     0x0d
#define CPLD1_SLAVE_ADDR        0x30
#define CPLD2_SLAVE_ADDR        0x31

static struct class*  fpgafwclass  = NULL; // < The device-driver class struct pointer
static struct device* fpgafwdev = NULL;    // < The device-driver device struct pointer

#define PCI_VENDOR_ID_TEST 0x1af4

#ifndef PCI_VENDOR_ID_XILINX
#define PCI_VENDOR_ID_XILINX 0x10EE
#endif

#define FPGA_PCIE_DEVICE_ID 0x7021
#define TEST_PCIE_DEVICE_ID 0x1110


#ifdef DEBUG_KERN
#define info(fmt,args...)  printk(KERN_INFO "line %3d : "fmt,__LINE__,##args)
#define check(REG)         printk(KERN_INFO "line %3d : %-8s = %2.2X",__LINE__,#REG,ioread8(REG));
#else
#define info(fmt,args...)
#define check(REG)
#endif

static struct mutex fpga_i2c_master_locks[I2C_MASTER_CH_TOTAL];
/* Store lasted switch address and channel */
static uint16_t fpga_i2c_lasted_access_port[I2C_MASTER_CH_TOTAL];

enum PORT_TYPE {
    NONE,
    QSFP,
    SFP
};

struct i2c_switch {
    unsigned char master_bus;   // I2C bus number
    unsigned char switch_addr;  // PCA9548 device address, 0xFF if directly connect to a bus.
    unsigned char channel;      // PCA9548 channel number. If the switch_addr is 0xFF, this value is ignored.
    enum PORT_TYPE port_type;   // QSFP/SFP tranceiver port type.
    char calling_name[20];      // Calling name.
};

struct i2c_dev_data {
    int portid;
    struct i2c_switch pca9548;
};

/* PREDEFINED I2C SWITCH DEVICE TOPOLOGY */
static struct i2c_switch fpga_i2c_bus_dev[] = {
    /* SFP and QSFP front panel I2C */
    {I2C_MASTER_CH_11, 0x70, 0, QSFP, "QSFP1"},   {I2C_MASTER_CH_11, 0x70, 1, QSFP, "QSFP2"},
    {I2C_MASTER_CH_11, 0x70, 2, QSFP, "QSFP3"},   {I2C_MASTER_CH_11, 0x70, 3, QSFP, "QSFP4"},
    {I2C_MASTER_CH_14, 0x73, 7, QSFP, "QSFP5"},   {I2C_MASTER_CH_14, 0x73, 6, QSFP, "QSFP6"},
    {I2C_MASTER_CH_14, 0x73, 5, QSFP, "QSFP7"},   {I2C_MASTER_CH_14, 0x73, 4, QSFP, "QSFP8"},
    {I2C_MASTER_CH_11, 0x70, 4, QSFP, "QSFP9"},   {I2C_MASTER_CH_11, 0x70, 5, QSFP, "QSFP10"},
    {I2C_MASTER_CH_11, 0x70, 6, QSFP, "QSFP11"},  {I2C_MASTER_CH_11, 0x70, 7, QSFP, "QSFP12"},
    {I2C_MASTER_CH_14, 0x73, 3, QSFP, "QSFP13"},  {I2C_MASTER_CH_14, 0x73, 2, QSFP, "QSFP14"},
    {I2C_MASTER_CH_14, 0x73, 1, QSFP, "QSFP15"},  {I2C_MASTER_CH_14, 0x73, 0, QSFP, "QSFP16"},

    {I2C_MASTER_CH_11, 0x71, 0, QSFP, "QSFP17"},  {I2C_MASTER_CH_11, 0x71, 1, QSFP, "QSFP18"},
    {I2C_MASTER_CH_11, 0x71, 2, QSFP, "QSFP19"},  {I2C_MASTER_CH_11, 0x71, 3, QSFP, "QSFP20"},
    {I2C_MASTER_CH_14, 0x72, 7, QSFP, "QSFP21"},  {I2C_MASTER_CH_14, 0x72, 6, QSFP, "QSFP22"},
    {I2C_MASTER_CH_14, 0x72, 5, QSFP, "QSFP23"},  {I2C_MASTER_CH_14, 0x72, 4, QSFP, "QSFP24"},
    {I2C_MASTER_CH_11, 0x71, 4, QSFP, "QSFP25"},  {I2C_MASTER_CH_11, 0x71, 5, QSFP, "QSFP26"},
    {I2C_MASTER_CH_11, 0x71, 6, QSFP, "QSFP27"},  {I2C_MASTER_CH_11, 0x71, 7, QSFP, "QSFP28"},
    {I2C_MASTER_CH_14, 0x72, 3, QSFP, "QSFP29"},  {I2C_MASTER_CH_14, 0x72, 2, QSFP, "QSFP30"},
    {I2C_MASTER_CH_14, 0x72, 1, QSFP, "QSFP31"},  {I2C_MASTER_CH_14, 0x72, 0, QSFP, "QSFP32"},

    {I2C_MASTER_CH_11, 0x72, 0, QSFP, "QSFP33"},  {I2C_MASTER_CH_11, 0x72, 1, QSFP, "QSFP34"},
    {I2C_MASTER_CH_11, 0x72, 2, QSFP, "QSFP35"},  {I2C_MASTER_CH_11, 0x72, 3, QSFP, "QSFP36"},
    {I2C_MASTER_CH_14, 0x71, 7, QSFP, "QSFP37"},  {I2C_MASTER_CH_14, 0x71, 6, QSFP, "QSFP38"},
    {I2C_MASTER_CH_14, 0x71, 5, QSFP, "QSFP39"},  {I2C_MASTER_CH_14, 0x71, 4, QSFP, "QSFP40"},
    {I2C_MASTER_CH_11, 0x72, 4, QSFP, "QSFP41"},  {I2C_MASTER_CH_11, 0x72, 5, QSFP, "QSFP42"},
    {I2C_MASTER_CH_11, 0x72, 6, QSFP, "QSFP43"},  {I2C_MASTER_CH_11, 0x72, 7, QSFP, "QSFP44"},
    {I2C_MASTER_CH_14, 0x71, 3, QSFP, "QSFP45"},  {I2C_MASTER_CH_14, 0x71, 2, QSFP, "QSFP46"},
    {I2C_MASTER_CH_14, 0x71, 1, QSFP, "QSFP47"},  {I2C_MASTER_CH_14, 0x71, 0, QSFP, "QSFP48"},

    {I2C_MASTER_CH_11, 0x73, 0, QSFP, "QSFP49"},  {I2C_MASTER_CH_11, 0x73, 1, QSFP, "QSFP50"},
    {I2C_MASTER_CH_11, 0x73, 2, QSFP, "QSFP51"},  {I2C_MASTER_CH_11, 0x73, 3, QSFP, "QSFP52"},
    {I2C_MASTER_CH_14, 0x70, 7, QSFP, "QSFP53"},  {I2C_MASTER_CH_14, 0x70, 6, QSFP, "QSFP54"},
    {I2C_MASTER_CH_14, 0x70, 5, QSFP, "QSFP55"},  {I2C_MASTER_CH_14, 0x70, 4, QSFP, "QSFP56"},
    {I2C_MASTER_CH_11, 0x73, 4, QSFP, "QSFP57"},  {I2C_MASTER_CH_11, 0x73, 5, QSFP, "QSFP58"},
    {I2C_MASTER_CH_11, 0x73, 6, QSFP, "QSFP59"},  {I2C_MASTER_CH_11, 0x73, 7, QSFP, "QSFP60"},
    {I2C_MASTER_CH_14, 0x70, 3, QSFP, "QSFP61"},  {I2C_MASTER_CH_14, 0x70, 2, QSFP, "QSFP62"},
    {I2C_MASTER_CH_14, 0x70, 1, QSFP, "QSFP63"},  {I2C_MASTER_CH_14, 0x70, 0, QSFP, "QSFP64"},

    {I2C_MASTER_CH_12, 0x70, 0, QSFP, "QSFP65"},  {I2C_MASTER_CH_12, 0x70, 1, QSFP, "QSFP66"},
    {I2C_MASTER_CH_12, 0x70, 2, QSFP, "QSFP67"},  {I2C_MASTER_CH_12, 0x70, 3, QSFP, "QSFP68"},
    {I2C_MASTER_CH_13, 0x73, 7, QSFP, "QSFP69"},  {I2C_MASTER_CH_13, 0x73, 6, QSFP, "QSFP70"},
    {I2C_MASTER_CH_13, 0x73, 5, QSFP, "QSFP71"},  {I2C_MASTER_CH_13, 0x73, 4, QSFP, "QSFP72"},
    {I2C_MASTER_CH_12, 0x70, 4, QSFP, "QSFP73"},  {I2C_MASTER_CH_12, 0x70, 5, QSFP, "QSFP74"},
    {I2C_MASTER_CH_12, 0x70, 6, QSFP, "QSFP75"},  {I2C_MASTER_CH_12, 0x70, 7, QSFP, "QSFP76"},
    {I2C_MASTER_CH_13, 0x73, 3, QSFP, "QSFP77"},  {I2C_MASTER_CH_13, 0x73, 2, QSFP, "QSFP78"},
    {I2C_MASTER_CH_13, 0x73, 1, QSFP, "QSFP79"},  {I2C_MASTER_CH_13, 0x73, 0, QSFP, "QSFP80"},

    {I2C_MASTER_CH_12, 0x71, 0, QSFP, "QSFP81"},  {I2C_MASTER_CH_12, 0x71, 1, QSFP, "QSFP82"},
    {I2C_MASTER_CH_12, 0x71, 2, QSFP, "QSFP83"},  {I2C_MASTER_CH_12, 0x71, 3, QSFP, "QSFP84"},
    {I2C_MASTER_CH_13, 0x72, 7, QSFP, "QSFP85"},  {I2C_MASTER_CH_13, 0x72, 6, QSFP, "QSFP86"},
    {I2C_MASTER_CH_13, 0x72, 5, QSFP, "QSFP87"},  {I2C_MASTER_CH_13, 0x72, 4, QSFP, "QSFP88"},
    {I2C_MASTER_CH_12, 0x71, 4, QSFP, "QSFP89"},  {I2C_MASTER_CH_12, 0x71, 5, QSFP, "QSFP90"},
    {I2C_MASTER_CH_12, 0x71, 6, QSFP, "QSFP91"},  {I2C_MASTER_CH_12, 0x71, 7, QSFP, "QSFP92"},
    {I2C_MASTER_CH_13, 0x72, 3, QSFP, "QSFP93"},  {I2C_MASTER_CH_13, 0x72, 2, QSFP, "QSFP94"},
    {I2C_MASTER_CH_13, 0x72, 1, QSFP, "QSFP95"},  {I2C_MASTER_CH_13, 0x72, 0, QSFP, "QSFP96"},

    {I2C_MASTER_CH_12, 0x72, 0, QSFP, "QSFP97"},  {I2C_MASTER_CH_12, 0x72, 1, QSFP, "QSFP98"}, 
    {I2C_MASTER_CH_12, 0x72, 2, QSFP, "QSFP99"},  {I2C_MASTER_CH_12, 0x72, 3, QSFP, "QSFP100"},
    {I2C_MASTER_CH_13, 0x71, 7, QSFP, "QSFP101"}, {I2C_MASTER_CH_13, 0x71, 6, QSFP, "QSFP102"},
    {I2C_MASTER_CH_13, 0x71, 5, QSFP, "QSFP103"}, {I2C_MASTER_CH_13, 0x71, 4, QSFP, "QSFP104"},
    {I2C_MASTER_CH_12, 0x72, 4, QSFP, "QSFP105"}, {I2C_MASTER_CH_12, 0x72, 5, QSFP, "QSFP106"},
    {I2C_MASTER_CH_12, 0x72, 6, QSFP, "QSFP107"}, {I2C_MASTER_CH_12, 0x72, 7, QSFP, "QSFP108"},
    {I2C_MASTER_CH_13, 0x71, 3, QSFP, "QSFP109"}, {I2C_MASTER_CH_13, 0x71, 2, QSFP, "QSFP110"},
    {I2C_MASTER_CH_13, 0x71, 1, QSFP, "QSFP111"}, {I2C_MASTER_CH_13, 0x71, 0, QSFP, "QSFP112"},

    {I2C_MASTER_CH_12, 0x73, 0, QSFP, "QSFP113"}, {I2C_MASTER_CH_12, 0x73, 1, QSFP, "QSFP114"},
    {I2C_MASTER_CH_12, 0x73, 2, QSFP, "QSFP115"}, {I2C_MASTER_CH_12, 0x73, 3, QSFP, "QSFP116"},
    {I2C_MASTER_CH_13, 0x70, 7, QSFP, "QSFP117"}, {I2C_MASTER_CH_13, 0x70, 6, QSFP, "QSFP118"},
    {I2C_MASTER_CH_13, 0x70, 5, QSFP, "QSFP119"}, {I2C_MASTER_CH_13, 0x70, 4, QSFP, "QSFP120"},
    {I2C_MASTER_CH_12, 0x73, 4, QSFP, "QSFP121"}, {I2C_MASTER_CH_12, 0x73, 5, QSFP, "QSFP122"},
    {I2C_MASTER_CH_12, 0x73, 6, QSFP, "QSFP123"}, {I2C_MASTER_CH_12, 0x73, 7, QSFP, "QSFP124"},
    {I2C_MASTER_CH_13, 0x70, 3, QSFP, "QSFP125"}, {I2C_MASTER_CH_13, 0x70, 2, QSFP, "QSFP126"},
    {I2C_MASTER_CH_13, 0x70, 1, QSFP, "QSFP127"}, {I2C_MASTER_CH_13, 0x70, 0, QSFP, "QSFP128"},

    /* Vritual I2C adapters */
    {I2C_MASTER_CH_1, 0xFF, 0, NONE, "I2C_1"}, // FAN
    {I2C_MASTER_CH_2, 0xFF, 0, NONE, "I2C_2"},
    {I2C_MASTER_CH_3, 0xFF, 0, NONE, "I2C_3"},
    {I2C_MASTER_CH_4, 0xFF, 0, NONE, "I2C_4"},
    {I2C_MASTER_CH_5, 0xFF, 0, NONE, "I2C_5"}, // BB
    {I2C_MASTER_CH_6, 0xFF, 0, NONE, "I2C_6"},
    {I2C_MASTER_CH_7, 0xFF, 0, NONE, "I2C_7"}, // SW[1-2]
    {I2C_MASTER_CH_8, 0xFF, 0, NONE, "I2C_8"}, // SW[3-4]

    // NOTE: Buses below are for front panel port debug     
    {I2C_MASTER_CH_11, 0xFF, 0, NONE, "I2C_11"},
    {I2C_MASTER_CH_12, 0xFF, 0, NONE, "I2C_12"},
    {I2C_MASTER_CH_13, 0xFF, 0, NONE, "I2C_13"},
    {I2C_MASTER_CH_14, 0xFF, 0, NONE, "I2C_14"},
};

#define VIRTUAL_I2C_PORT_LENGTH ARRAY_SIZE(fpga_i2c_bus_dev)
#define FAN_I2C_CPLD_INDEX      SFF_PORT_TOTAL
#define BB_I2C_CPLD_INDEX       SFF_PORT_TOTAL + 4
#define SW1_I2C_CPLD_INDEX      SFF_PORT_TOTAL + 6
#define SW2_I2C_CPLD_INDEX      SFF_PORT_TOTAL + 7

struct fpga_device {
    /* data mmio region */
    void __iomem *data_base_addr;
    resource_size_t data_mmio_start;
    resource_size_t data_mmio_len;
};

static struct fpga_device fpga_dev = {
    .data_base_addr = 0,
    .data_mmio_start = 0,
    .data_mmio_len = 0,
};

struct phalanx_fpga_data {
    struct device *sff_devices[SFF_PORT_TOTAL];
    struct i2c_client *sff_i2c_clients[SFF_PORT_TOTAL];
    struct i2c_adapter *i2c_adapter[VIRTUAL_I2C_PORT_LENGTH];
    struct mutex fpga_lock;         // For FPGA internal lock
    void __iomem * fpga_read_addr;
    uint8_t cpld1_read_addr;
    uint8_t cpld2_read_addr;
    uint8_t cpld3_read_addr;
    uint8_t cpld4_read_addr;
    uint8_t fancpld_read_addr;
};

struct sff_device_data {
    int portid;
    enum PORT_TYPE port_type;
};

struct phalanx_fpga_data *fpga_data;

/*
 * Kernel object for other module drivers.
 * Other module can use these kobject as a parent.
 */

static struct kobject *fpga = NULL;
static struct kobject *cpld1 = NULL;
static struct kobject *cpld2 = NULL;
static struct kobject *cpld3 = NULL;
static struct kobject *cpld4 = NULL;
static struct kobject *fancpld = NULL;

/**
 * Device node in sysfs tree.
 */
static struct device *sff_dev = NULL;

/**
 * Show the value of the register set by 'set_fpga_reg_address'
 * If the address is not set by 'set_fpga_reg_address' first,
 * The version register is selected by default.
 * @param  buf     register value in hextring
 * @return         number of bytes read, or an error code
 */
static ssize_t get_fpga_reg_value(struct device *dev, struct device_attribute *devattr,
                                  char *buf)
{
    // read data from the address
    uint32_t data;
    data = ioread32(fpga_data->fpga_read_addr);
    return sprintf(buf, "0x%8.8x\n", data);
}
/**
 * Store the register address
 * @param  buf     address wanted to be read value of
 * @return         number of bytes stored, or an error code
 */
static ssize_t set_fpga_reg_address(struct device *dev, struct device_attribute *devattr,
                                    const char *buf, size_t count)
{
    uint32_t addr;
    char *last;

    addr = (uint32_t)strtoul(buf, &last, 16);
    if (addr == 0 && buf == last) {
        return -EINVAL;
    }
    fpga_data->fpga_read_addr = fpga_dev.data_base_addr + addr;
    return count;
}
/**
 * Show value of fpga scratch register
 * @param  buf     register value in hexstring
 * @return         number of bytes read, or an error code
 */
static ssize_t get_fpga_scratch(struct device *dev, struct device_attribute *devattr,
                                char *buf)
{
    return sprintf(buf, "0x%8.8x\n", ioread32(fpga_dev.data_base_addr + FPGA_SCRATCH) & 0xffffffff);
}
/**
 * Store value of fpga scratch register
 * @param  buf     scratch register value passing from user space
 * @return         number of bytes stored, or an error code
 */
static ssize_t set_fpga_scratch(struct device *dev, struct device_attribute *devattr,
                                const char *buf, size_t count)
{
    uint32_t data;
    char *last;
    data = (uint32_t)strtoul(buf, &last, 16);
    if (data == 0 && buf == last) {
        return -EINVAL;
    }
    iowrite32(data, fpga_dev.data_base_addr + FPGA_SCRATCH);
    return count;
}
/**
 * Store a value in a specific register address
 * @param  buf     the value and address in format '0xhhhh 0xhhhhhhhh'
 * @return         number of bytes sent by user space, or an error code
 */
static ssize_t set_fpga_reg_value(struct device *dev, struct device_attribute *devattr,
                                  const char *buf, size_t count)
{
    // register are 4 bytes
    uint32_t addr;
    uint32_t value;
    uint32_t mode = 8;
    char *tok;
    char clone[count];
    char *pclone = clone;
    char *last;

    strcpy(clone, buf);

    mutex_lock(&fpga_data->fpga_lock);
    tok = strsep((char**)&pclone, " ");
    if (tok == NULL) {
        mutex_unlock(&fpga_data->fpga_lock);
        return -EINVAL;
    }
    addr = (uint32_t)strtoul(tok, &last, 16);
    if (addr == 0 && tok == last) {
        mutex_unlock(&fpga_data->fpga_lock);
        return -EINVAL;
    }
    tok = strsep((char**)&pclone, " ");
    if (tok == NULL) {
        mutex_unlock(&fpga_data->fpga_lock);
        return -EINVAL;
    }
    value = (uint32_t)strtoul(tok, &last, 16);
    if (value == 0 && tok == last) {
        mutex_unlock(&fpga_data->fpga_lock);
        return -EINVAL;
    }
    tok = strsep((char**)&pclone, " ");
    if (tok == NULL) {
        mode = 32;
    } else {
        mode = (uint32_t)strtoul(tok, &last, 10);
        if (mode == 0 && tok == last) {
            mutex_unlock(&fpga_data->fpga_lock);
            return -EINVAL;
        }
    }
    if (mode == 32) {
        iowrite32(value, fpga_dev.data_base_addr + addr);
    } else if (mode == 8) {
        iowrite8(value, fpga_dev.data_base_addr + addr);
    } else {
        mutex_unlock(&fpga_data->fpga_lock);
        return -EINVAL;
    }
    mutex_unlock(&fpga_data->fpga_lock);
    return count;
}

/* FPGA attributes */
static DEVICE_ATTR( getreg, 0600, get_fpga_reg_value, set_fpga_reg_address);
static DEVICE_ATTR( scratch, 0600, get_fpga_scratch, set_fpga_scratch);
static DEVICE_ATTR( setreg, 0200, NULL , set_fpga_reg_value);

static struct attribute *fpga_attrs[] = {
    &dev_attr_getreg.attr,
    &dev_attr_scratch.attr,
    &dev_attr_setreg.attr,
    NULL,
};

static struct attribute_group fpga_attr_grp = {
    .attrs = fpga_attrs,
};

/* SW CPLDs attributes */
static ssize_t cpld1_getreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    // CPLD register is one byte
    uint8_t data;
    fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, fpga_data->cpld1_read_addr, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&data);
    return sprintf(buf, "0x%2.2x\n", data);
}
static ssize_t cpld1_getreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    uint8_t addr;
    char *last;
    addr = (uint8_t)strtoul(buf, &last, 16);
    if (addr == 0 && buf == last) {
        return -EINVAL;
    }
    fpga_data->cpld1_read_addr = addr;
    return size;
}
struct device_attribute dev_attr_cpld1_getreg = __ATTR(getreg, 0600, cpld1_getreg_show, cpld1_getreg_store);

static ssize_t cpld1_scratch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    // CPLD register is one byte
    __u8 data;
    int err;
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, 0x01, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&data);
    if (err < 0)
        return err;
    return sprintf(buf, "0x%2.2x\n", data);
}
static ssize_t cpld1_scratch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    // CPLD register is one byte
    __u8 data;
    char *last;
    int err;
    data = (uint8_t)strtoul(buf, &last, 16);
    if (data == 0 && buf == last) {
        return -EINVAL;
    }
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00, I2C_SMBUS_WRITE, 0x01, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&data);
    if (err < 0)
        return err;
    return size;
}
struct device_attribute dev_attr_cpld1_scratch = __ATTR(scratch, 0600, cpld1_scratch_show, cpld1_scratch_store);

static ssize_t cpld1_setreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{

    uint8_t addr, value;
    char *tok;
    char clone[size];
    char *pclone = clone;
    int err;
    char *last;

    strcpy(clone, buf);

    tok = strsep((char**)&pclone, " ");
    if (tok == NULL) {
        return -EINVAL;
    }
    addr = (uint8_t)strtoul(tok, &last, 16);
    if (addr == 0 && tok == last) {
        return -EINVAL;
    }
    tok = strsep((char**)&pclone, " ");
    if (tok == NULL) {
        return -EINVAL;
    }
    value = (uint8_t)strtoul(tok, &last, 16);
    if (value == 0 && tok == last) {
        return -EINVAL;
    }

    err = fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00, I2C_SMBUS_WRITE, addr, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&value);
    if (err < 0)
        return err;

    return size;
}
struct device_attribute dev_attr_cpld1_setreg = __ATTR(setreg, 0200, NULL, cpld1_setreg_store);

static struct attribute *cpld1_attrs[] = {
    &dev_attr_cpld1_getreg.attr,
    &dev_attr_cpld1_scratch.attr,
    &dev_attr_cpld1_setreg.attr,
    NULL,
};

static struct attribute_group cpld1_attr_grp = {
    .attrs = cpld1_attrs,
};

static ssize_t cpld2_getreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    // CPLD register is one byte
    uint8_t data;
    fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, fpga_data->cpld2_read_addr, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&data);
    return sprintf(buf, "0x%2.2x\n", data);
}

static ssize_t cpld2_getreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    // CPLD register is one byte
    uint32_t addr;
    char *last;
    addr = (uint8_t)strtoul(buf, &last, 16);
    if (addr == 0 && buf == last) {
        return -EINVAL;
    }
    fpga_data->cpld2_read_addr = addr;
    return size;
}
struct device_attribute dev_attr_cpld2_getreg = __ATTR(getreg, 0600, cpld2_getreg_show, cpld2_getreg_store);

static ssize_t cpld2_scratch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    // CPLD register is one byte
    __u8 data;
    int err;
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, 0x01, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&data);
    if (err < 0)
        return err;
    return sprintf(buf, "0x%2.2x\n", data);
}

static ssize_t cpld2_scratch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    // CPLD register is one byte
    __u8 data;
    char *last;
    int err;

    data = (uint8_t)strtoul(buf, &last, 16);
    if (data == 0 && buf == last) {
        return -EINVAL;
    }
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00, I2C_SMBUS_WRITE, 0x01, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&data);
    if (err < 0)
        return err;
    return size;
}
struct device_attribute dev_attr_cpld2_scratch = __ATTR(scratch, 0600, cpld2_scratch_show, cpld2_scratch_store);

static ssize_t cpld2_setreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    uint8_t addr, value;
    char *tok;
    char clone[size];
    char *pclone = clone;
    int err;
    char *last;

    strcpy(clone, buf);

    tok = strsep((char**)&pclone, " ");
    if (tok == NULL) {
        return -EINVAL;
    }
    addr = (uint8_t)strtoul(tok, &last, 16);
    if (addr == 0 && tok == last) {
        return -EINVAL;
    }
    tok = strsep((char**)&pclone, " ");
    if (tok == NULL) {
        return -EINVAL;
    }
    value = (uint8_t)strtoul(tok, &last, 16);
    if (value == 0 && tok == last) {
        return -EINVAL;
    }

    err = fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00, I2C_SMBUS_WRITE, addr, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&value);
    if (err < 0)
        return err;

    return size;
}
struct device_attribute dev_attr_cpld2_setreg = __ATTR(setreg, 0200, NULL, cpld2_setreg_store);

static struct attribute *cpld2_attrs[] = {
    &dev_attr_cpld2_getreg.attr,
    &dev_attr_cpld2_scratch.attr,
    &dev_attr_cpld2_setreg.attr,
    NULL,
};

static struct attribute_group cpld2_attr_grp = {
    .attrs = cpld2_attrs,
};

static ssize_t cpld3_getreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    // CPLD register is one byte
    uint8_t data;
    fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, fpga_data->cpld3_read_addr, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&data);
    return sprintf(buf, "0x%2.2x\n", data);
}

static ssize_t cpld3_getreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    // CPLD register is one byte
    uint32_t addr;
    char *last;
    addr = (uint8_t)strtoul(buf, &last, 16);
    if (addr == 0 && buf == last) {
        return -EINVAL;
    }
    fpga_data->cpld3_read_addr = addr;
    return size;
}
struct device_attribute dev_attr_cpld3_getreg = __ATTR(getreg, 0600, cpld3_getreg_show, cpld3_getreg_store);

static ssize_t cpld3_scratch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    // CPLD register is one byte
    __u8 data;
    int err;
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, 0x01, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&data);
    if (err < 0)
        return err;
    return sprintf(buf, "0x%2.2x\n", data);
}

static ssize_t cpld3_scratch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    // CPLD register is one byte
    __u8 data;
    char *last;
    int err;

    data = (uint8_t)strtoul(buf, &last, 16);
    if (data == 0 && buf == last) {
        return -EINVAL;
    }
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00, I2C_SMBUS_WRITE, 0x01, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&data);
    if (err < 0)
        return err;
    return size;
}
struct device_attribute dev_attr_cpld3_scratch = __ATTR(scratch, 0600, cpld3_scratch_show, cpld3_scratch_store);

static ssize_t cpld3_setreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    uint8_t addr, value;
    char *tok;
    char clone[size];
    char *pclone = clone;
    int err;
    char *last;

    strcpy(clone, buf);

    tok = strsep((char**)&pclone, " ");
    if (tok == NULL) {
        return -EINVAL;
    }
    addr = (uint8_t)strtoul(tok, &last, 16);
    if (addr == 0 && tok == last) {
        return -EINVAL;
    }
    tok = strsep((char**)&pclone, " ");
    if (tok == NULL) {
        return -EINVAL;
    }
    value = (uint8_t)strtoul(tok, &last, 16);
    if (value == 0 && tok == last) {
        return -EINVAL;
    }

    err = fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00, I2C_SMBUS_WRITE, addr, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&value);
    if (err < 0)
        return err;

    return size;
}
struct device_attribute dev_attr_cpld3_setreg = __ATTR(setreg, 0200, NULL, cpld3_setreg_store);

static struct attribute *cpld3_attrs[] = {
    &dev_attr_cpld3_getreg.attr,
    &dev_attr_cpld3_scratch.attr,
    &dev_attr_cpld3_setreg.attr,
    NULL,
};

static struct attribute_group cpld3_attr_grp = {
    .attrs = cpld3_attrs,
};

static ssize_t cpld4_getreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    // CPLD register is one byte
    uint8_t data;
    fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, fpga_data->cpld4_read_addr, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&data);
    return sprintf(buf, "0x%2.2x\n", data);
}

static ssize_t cpld4_getreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    // CPLD register is one byte
    uint32_t addr;
    char *last;
    addr = (uint8_t)strtoul(buf, &last, 16);
    if (addr == 0 && buf == last) {
        return -EINVAL;
    }
    fpga_data->cpld4_read_addr = addr;
    return size;
}
struct device_attribute dev_attr_cpld4_getreg = __ATTR(getreg, 0600, cpld4_getreg_show, cpld4_getreg_store);

static ssize_t cpld4_scratch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    // CPLD register is one byte
    __u8 data;
    int err;
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, 0x01, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&data);
    if (err < 0)
        return err;
    return sprintf(buf, "0x%2.2x\n", data);
}

static ssize_t cpld4_scratch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    // CPLD register is one byte
    __u8 data;
    char *last;
    int err;

    data = (uint8_t)strtoul(buf, &last, 16);
    if (data == 0 && buf == last) {
        return -EINVAL;
    }
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00, I2C_SMBUS_WRITE, 0x01, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&data);
    if (err < 0)
        return err;
    return size;
}
struct device_attribute dev_attr_cpld4_scratch = __ATTR(scratch, 0600, cpld4_scratch_show, cpld4_scratch_store);

static ssize_t cpld4_setreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    uint8_t addr, value;
    char *tok;
    char clone[size];
    char *pclone = clone;
    int err;
    char *last;

    strcpy(clone, buf);

    tok = strsep((char**)&pclone, " ");
    if (tok == NULL) {
        return -EINVAL;
    }
    addr = (uint8_t)strtoul(tok, &last, 16);
    if (addr == 0 && tok == last) {
        return -EINVAL;
    }
    tok = strsep((char**)&pclone, " ");
    if (tok == NULL) {
        return -EINVAL;
    }
    value = (uint8_t)strtoul(tok, &last, 16);
    if (value == 0 && tok == last) {
        return -EINVAL;
    }

    err = fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00, I2C_SMBUS_WRITE, addr, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&value);
    if (err < 0)
        return err;

    return size;
}
struct device_attribute dev_attr_cpld4_setreg = __ATTR(setreg, 0200, NULL, cpld4_setreg_store);

static struct attribute *cpld4_attrs[] = {
    &dev_attr_cpld4_getreg.attr,
    &dev_attr_cpld4_scratch.attr,
    &dev_attr_cpld4_setreg.attr,
    NULL,
};

static struct attribute_group cpld4_attr_grp = {
    .attrs = cpld4_attrs,
};

/* FAN CPLD */
static ssize_t fancpld_getreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    // CPLD register is one byte
    uint8_t data;
    fpga_i2c_access(fpga_data->i2c_adapter[FAN_I2C_CPLD_INDEX], FAN_CPLD_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, fpga_data->fancpld_read_addr, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&data);
    return sprintf(buf, "0x%2.2x\n", data);
}
static ssize_t fancpld_getreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    uint8_t addr;
    char *last;
    addr = (uint8_t)strtoul(buf, &last, 16);
    if (addr == 0 && buf == last) {
        return -EINVAL;
    }
    fpga_data->fancpld_read_addr = addr;
    return size;
}
struct device_attribute dev_attr_fancpld_getreg = __ATTR(getreg, 0600, fancpld_getreg_show, fancpld_getreg_store);

static ssize_t fancpld_scratch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    // CPLD register is one byte
    __u8 data;
    int err;
    err = fpga_i2c_access(fpga_data->i2c_adapter[FAN_I2C_CPLD_INDEX], FAN_CPLD_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, 0x04, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&data);
    if (err < 0)
        return err;
    return sprintf(buf, "0x%2.2x\n", data);
}
static ssize_t fancpld_scratch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    // CPLD register is one byte
    __u8 data;
    char *last;
    int err;
    data = (uint8_t)strtoul(buf, &last, 16);
    if (data == 0 && buf == last) {
        return -EINVAL;
    }
    err = fpga_i2c_access(fpga_data->i2c_adapter[FAN_I2C_CPLD_INDEX], FAN_CPLD_SLAVE_ADDR, 0x00, I2C_SMBUS_WRITE, 0x04, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&data);
    if (err < 0)
        return err;
    return size;
}
struct device_attribute dev_attr_fancpld_scratch = __ATTR(scratch, 0600, fancpld_scratch_show, fancpld_scratch_store);

static ssize_t fancpld_setreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{

    uint8_t addr, value;
    char *tok;
    char clone[size];
    char *pclone = clone;
    int err;
    char *last;

    strcpy(clone, buf);

    tok = strsep((char**)&pclone, " ");
    if (tok == NULL) {
        return -EINVAL;
    }
    addr = (uint8_t)strtoul(tok, &last, 16);
    if (addr == 0 && tok == last) {
        return -EINVAL;
    }
    tok = strsep((char**)&pclone, " ");
    if (tok == NULL) {
        return -EINVAL;
    }
    value = (uint8_t)strtoul(tok, &last, 16);
    if (value == 0 && tok == last) {
        return -EINVAL;
    }

    err = fpga_i2c_access(fpga_data->i2c_adapter[FAN_I2C_CPLD_INDEX], FAN_CPLD_SLAVE_ADDR, 0x00, I2C_SMBUS_WRITE, addr, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&value);
    if (err < 0)
        return err;

    return size;
}
struct device_attribute dev_attr_fancpld_setreg = __ATTR(setreg, 0200, NULL, fancpld_setreg_store);

static struct attribute *fancpld_attrs[] = {
    &dev_attr_fancpld_getreg.attr,
    &dev_attr_fancpld_scratch.attr,
    &dev_attr_fancpld_setreg.attr,
    NULL,
};

static struct attribute_group fancpld_attr_grp = {
    .attrs = fancpld_attrs,
};

static struct attribute *fancpld_no_attrs[] = {
    NULL,
};

static struct attribute_group fancpld_no_attr_grp = {
    .attrs = fancpld_no_attrs,
};

/* QSFP/SFP+ attributes */
static ssize_t qsfp_modirq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 data;
    int err;
    struct sff_device_data *dev_data = dev_get_drvdata(dev);
    unsigned int portid = dev_data->portid;
    err = i2c_xcvr_access(I2C_XCVR_STAT,portid,&data,I2C_SMBUS_READ);
    if(err < 0){
        return err;
    }
    return sprintf(buf, "%d\n", (data >> I2C_STAT_INT_N) & 1U);
}
DEVICE_ATTR_RO(qsfp_modirq);

static ssize_t qsfp_modprs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 data;
    int err;
    struct sff_device_data *dev_data = dev_get_drvdata(dev);
    unsigned int portid = dev_data->portid;
    err = i2c_xcvr_access(I2C_XCVR_STAT,portid,&data,I2C_SMBUS_READ);
    if(err < 0){
        return err;
    }
    return sprintf(buf, "%d\n", (data >> I2C_STAT_MODABS) & 1U);
}
DEVICE_ATTR_RO(qsfp_modprs);

static ssize_t sfp_txfault_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 data;
    int err;
    struct sff_device_data *dev_data = dev_get_drvdata(dev);
    unsigned int portid = dev_data->portid;
    err = i2c_xcvr_access(I2C_XCVR_STAT,portid,&data,I2C_SMBUS_READ);
    if(err < 0){
        return err;
    }
    return sprintf(buf, "%d\n", (data >> I2C_STAT_TXFAULT) & 1U);
}
DEVICE_ATTR_RO(sfp_txfault);

static ssize_t sfp_rxlos_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 data;
    int err;
    struct sff_device_data *dev_data = dev_get_drvdata(dev);
    unsigned int portid = dev_data->portid;
    err = i2c_xcvr_access(I2C_XCVR_STAT,portid,&data,I2C_SMBUS_READ);
    if(err < 0){
        return err;
    }
    return sprintf(buf, "%d\n", (data >> I2C_STAT_RXLOS) & 1U);
}
DEVICE_ATTR_RO(sfp_rxlos);

static ssize_t sfp_modabs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 data;
    int err;
    struct sff_device_data *dev_data = dev_get_drvdata(dev);
    unsigned int portid = dev_data->portid;
    err = i2c_xcvr_access(I2C_XCVR_STAT,portid,&data,I2C_SMBUS_READ);
    if(err < 0){
        return err;
    }
    return sprintf(buf, "%d\n", (data >> I2C_STAT_MODABS) & 1U);
}
DEVICE_ATTR_RO(sfp_modabs);

static ssize_t qsfp_modsel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 data;
    int err;
    struct sff_device_data *dev_data = dev_get_drvdata(dev);
    unsigned int portid = dev_data->portid;
    err = i2c_xcvr_access(I2C_XCVR_CTRL,portid,&data,I2C_SMBUS_READ);
    if(err < 0){
        return err;
    }
    return sprintf(buf, "%d\n", (data >> I2C_CTRL_MODSEL) & 1U);
}
static ssize_t qsfp_modsel_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    ssize_t status;
    long value;
    u8 data;
    struct sff_device_data *dev_data = dev_get_drvdata(dev);
    unsigned int portid = dev_data->portid;

    status = kstrtol(buf, 0, &value);
    if (status == 0) {
        // if value is 0, clear bit.
        i2c_xcvr_access(I2C_XCVR_CTRL,portid,&data,I2C_SMBUS_READ);
        if (!value)
            data = data & ~( 1U << I2C_CTRL_MODSEL );
        else
            data = data | ( 1U << I2C_CTRL_MODSEL );
        i2c_xcvr_access(I2C_XCVR_CTRL,portid,&data,I2C_SMBUS_WRITE);
        status = size;
    }
    return status;
}
DEVICE_ATTR_RW(qsfp_modsel);

static ssize_t qsfp_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 data;
    int err;
    struct sff_device_data *dev_data = dev_get_drvdata(dev);
    unsigned int portid = dev_data->portid;
    err = i2c_xcvr_access(I2C_XCVR_CTRL,portid,&data,I2C_SMBUS_READ);
    if(err < 0){
        return err;
    }
    return sprintf(buf, "%d\n", (data >> I2C_CTRL_RST) & 1U);
}

static ssize_t qsfp_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    ssize_t status;
    long value;
    u8 data;
    struct sff_device_data *dev_data = dev_get_drvdata(dev);
    unsigned int portid = dev_data->portid;

    status = kstrtol(buf, 0, &value);
    if (status == 0) {
        // if value is 0, reset signal is low
        i2c_xcvr_access(I2C_XCVR_CTRL,portid,&data,I2C_SMBUS_READ);
        if (!value)
            data = data & ~((u8)0x1 << I2C_CTRL_RST);
        else
            data = data | ((u8)0x1 << I2C_CTRL_RST);
        i2c_xcvr_access(I2C_XCVR_CTRL,portid,&data,I2C_SMBUS_WRITE);
        status = size;
    }
    return status;
}
DEVICE_ATTR_RW(qsfp_reset);

static ssize_t sfp_txdisable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 data;
    int err;
    struct sff_device_data *dev_data = dev_get_drvdata(dev);
    unsigned int portid = dev_data->portid;
    err = i2c_xcvr_access(I2C_XCVR_CTRL,portid,&data,I2C_SMBUS_READ);
    if(err < 0){
        return err;
    }
    return sprintf(buf, "%d\n", (data >> I2C_CTRL_TXDIS) & 1U);
}
static ssize_t sfp_txdisable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    ssize_t status;
    long value;
    u8 data;
    struct sff_device_data *dev_data = dev_get_drvdata(dev);
    unsigned int portid = dev_data->portid;

    mutex_lock(&fpga_data->fpga_lock);
    status = kstrtol(buf, 0, &value);
    if (status == 0) {
        // check if value is 0 clear
        i2c_xcvr_access(I2C_XCVR_CTRL,portid,&data,I2C_SMBUS_READ);
        if (!value)
            data = data & ~((u8)0x1 << I2C_CTRL_TXDIS);
        else
            data = data | ((u8)0x1 << I2C_CTRL_TXDIS);
        i2c_xcvr_access(I2C_XCVR_CTRL,portid,&data,I2C_SMBUS_WRITE);
        status = size;
    }
    mutex_unlock(&fpga_data->fpga_lock);
    return status;
}
DEVICE_ATTR_RW(sfp_txdisable);

static struct attribute *sff_attrs[] = {
    &dev_attr_qsfp_modirq.attr,
    &dev_attr_qsfp_modprs.attr,
    &dev_attr_qsfp_modsel.attr,
    &dev_attr_qsfp_reset.attr,
    &dev_attr_sfp_txfault.attr,
    &dev_attr_sfp_rxlos.attr,
    &dev_attr_sfp_modabs.attr,
    &dev_attr_sfp_txdisable.attr,
    NULL,
};

static struct attribute_group sff_attr_grp = {
    .attrs = sff_attrs,
};

static const struct attribute_group *sff_attr_grps[] = {
    &sff_attr_grp,
    NULL
};


static ssize_t port_led_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    // value can be "nomal", "test"
    __u8 led_mode_1, led_mode_2, led_mode_3, led_mode_4;
    int err;
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, 0x09, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_mode_1);
    if (err < 0)
        return err;
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, 0x09, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_mode_2);
    if (err < 0)
        return err;
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, 0x09, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_mode_3);
    if (err < 0)
        return err;
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, 0x09, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_mode_4);
    if (err < 0)
        return err;
    return sprintf(buf, "%s %s %s %s\n",
                   led_mode_1 ? "test" : "normal",
                   led_mode_2 ? "test" : "normal",
                   led_mode_3 ? "test" : "normal",
                   led_mode_4 ? "test" : "normal");
}
static ssize_t port_led_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    int status;
    __u8 led_mode_1;
    if (sysfs_streq(buf, "test")) {
        led_mode_1 = 0x01;
    } else if (sysfs_streq(buf, "normal")) {
        led_mode_1 = 0x00;
    } else {
        return -EINVAL;
    }
    status = fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00,
                             I2C_SMBUS_WRITE, 0x09, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_mode_1);
    status = fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00,
                             I2C_SMBUS_WRITE, 0x09, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_mode_1);
    status = fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00,
                             I2C_SMBUS_WRITE, 0x09, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_mode_1);
    status = fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00,
                             I2C_SMBUS_WRITE, 0x09, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_mode_1);
    return size;
}
DEVICE_ATTR_RW(port_led_mode);

// Only work when port_led_mode set to 1
static ssize_t port_led_color_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    // value can be green/amber/both/alt-blink/OFF
    __u8 led_color1, led_color2, led_color3, led_color4;
    int err;
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, 0x09, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_color1);
    if (err < 0)
        return err;
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, 0x09, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_color2);
    if (err < 0)
        return err;
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, 0x09, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_color3);
    if (err < 0)
        return err;
    err = fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00, I2C_SMBUS_READ, 0x09, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_color4);
    if (err < 0)
        return err;
    return sprintf(buf, "%s %s %s %s\n",
                   led_color1 == 0x07 ? "off" : led_color1 == 0x06 ? "green" : led_color1 == 0x05 ?  "amber" : led_color1 == 0x04 ? 
                    "both" : "alt-blink",
                   led_color1 == 0x07 ? "off" : led_color1 == 0x06 ? "green" : led_color1 == 0x05 ?  "amber" : led_color1 == 0x04 ? 
                    "both" : "alt-blink",
                   led_color1 == 0x07 ? "off" : led_color1 == 0x06 ? "green" : led_color1 == 0x05 ?  "amber" : led_color1 == 0x04 ? 
                    "both" : "alt-blink",
                   led_color1 == 0x07 ? "off" : led_color1 == 0x06 ? "green" : led_color1 == 0x05 ?  "amber" : led_color1 == 0x04 ? 
                    "both" : "alt-blink");
}

static ssize_t port_led_color_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    int status;
    __u8 led_color;
    if (sysfs_streq(buf, "off")) {
        led_color = 0x07;
    } else if (sysfs_streq(buf, "green")) {
        led_color = 0x06;
    } else if (sysfs_streq(buf, "amber")) {
        led_color = 0x05;
    } else if (sysfs_streq(buf, "both")) {
        led_color = 0x04;
    } else if (sysfs_streq(buf, "alt-blink")) {
        led_color = 0x03;
    } else {
        status = -EINVAL;
        return status;
    }
    status = fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00,
                             I2C_SMBUS_WRITE, 0x0A, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_color);
    status = fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00,
                             I2C_SMBUS_WRITE, 0x0A, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_color);
    status = fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00,
                             I2C_SMBUS_WRITE, 0x0A, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_color);
    status = fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00,
                             I2C_SMBUS_WRITE, 0x0A, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&led_color);
    return size;
}
DEVICE_ATTR_RW(port_led_color);

static struct attribute *sff_led_test[] = {
    &dev_attr_port_led_mode.attr,
    &dev_attr_port_led_color.attr,
    NULL,
};

static struct attribute_group sff_led_test_grp = {
    .attrs = sff_led_test,
};

static struct device * phalanx_sff_init(int portid) {
    struct sff_device_data *new_data;
    struct device *new_device;

    new_data = kzalloc(sizeof(*new_data), GFP_KERNEL);
    if (!new_data) {
        printk(KERN_ALERT "Cannot alloc sff device data @port%d", portid);
        return NULL;
    }
    /* The QSFP port ID start from 1 */
    new_data->portid = portid + 1;
    new_data->port_type = fpga_i2c_bus_dev[portid].port_type;
    new_device = device_create_with_groups(fpgafwclass, sff_dev, MKDEV(0, 0), new_data, sff_attr_grps, "%s", fpga_i2c_bus_dev[portid].calling_name);
    if (IS_ERR(new_device)) {
        printk(KERN_ALERT "Cannot create sff device @port%d", portid);
        kfree(new_data);
        return NULL;
    }
    return new_device;
}

static int i2c_core_init(unsigned int master_bus, unsigned int freq_div,void __iomem *pci_bar){

    unsigned int ctrl;
    unsigned int REG_FREQ_L;
    unsigned int REG_FREQ_H;
    unsigned int REG_CTRL;
    unsigned int REG_CMD;

    REG_FREQ_L = I2C_MASTER_FREQ_L  + (master_bus - 1) * 0x20;
    REG_FREQ_H = I2C_MASTER_FREQ_H  + (master_bus - 1) * 0x20;
    REG_CTRL   = I2C_MASTER_CTRL    + (master_bus - 1) * 0x20;
    REG_CMD    = I2C_MASTER_CMD     + (master_bus - 1) * 0x20;

    if ( freq_div != I2C_DIV_100K ) {
        printk(KERN_ERR "FPGA I2C core: Unsupported clock divider: %x\n", freq_div);
        return -EINVAL;
    }

    // Makes sure core is disable
    ctrl = ioread8(pci_bar + REG_CTRL);
    iowrite8( ctrl & ~(1 << I2C_CTRL_EN | 1 << I2C_CTRL_IEN), pci_bar + REG_CTRL);
    iowrite8( freq_div & 0xFF , pci_bar + REG_FREQ_L);
    iowrite8( freq_div >> 8, pci_bar + REG_FREQ_H);
    iowrite8(1 << I2C_CMD_IACK, pci_bar + REG_CMD);
    iowrite8(1 << I2C_CTRL_EN | 1 << I2C_CTRL_IEN, pci_bar + REG_CTRL);

    return 0;
}

static void i2c_core_deinit(unsigned int master_bus,void __iomem *pci_bar){

    unsigned int REG_CTRL;
    REG_CTRL = I2C_MASTER_CTRL + (master_bus - 1) * 0x20;
    // Disable core
    iowrite8( ioread8(pci_bar + REG_CTRL) & ~(1 << I2C_CTRL_EN| 1 << I2C_CTRL_IEN), pci_bar + REG_CTRL);
}

/**
 * We have 2 line cards and each with 2 cplds here.
 * Each line card have 64 ports. Each cpld manage 32 ports.
 * protid start from 1.
 *  +---+------------------+------------------+
 *  |LC1|1     CPLD1     57|65   CPLD2     121|
 *  |   |4               60|68             124|
 *  +---+------------------+------------------+
 *  |LC2|5     CPLD1     61|69   CPLD2     125|
 *  |   |8               64|72             128|
 *  +---+------------------+------------------+
 */
static int i2c_xcvr_access(u8 register_address, unsigned int portid, u8 *data, char rw){
    
    u16 dev_addr = 0;
    int err;
    int lci, cpldi, i2c_adapter_index;
    int row, col;

    /* check for portid valid length */
    if(portid < 1 || portid > SFF_PORT_TOTAL){
        return -EINVAL;
    }

    lci = ( portid -1 ) % 8;
    cpldi = (portid -1 ) / 8;

    /* Normalize the port xposition into 4 x 8 metric.
     * Then calculate the cpld's port index from matrix.
     */
    row = (portid -1 ) % 4;
    col = (portid -1 ) / 8;

    // Line card select top/buttom
    if( lci < 4 ){
        i2c_adapter_index = SW1_I2C_CPLD_INDEX;
    }else{
        i2c_adapter_index = SW2_I2C_CPLD_INDEX;
    }

    // CPLD select left/right
    if( cpldi < 8 ){
        dev_addr = CPLD1_SLAVE_ADDR;
    }else{
        col -= 8;
        dev_addr = CPLD2_SLAVE_ADDR;
    }
    
    // Calculate cpld portid
    portid = ( 4 * col ) + row + 1;

    // Select port
    err = fpga_i2c_access(fpga_data->i2c_adapter[i2c_adapter_index], dev_addr, 0x00, I2C_SMBUS_WRITE, 
        I2C_XCVR_SEL, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&portid);
    if(err < 0){
        return err;
    }
    // Read/write port xcvr register
    err = fpga_i2c_access(fpga_data->i2c_adapter[i2c_adapter_index], dev_addr, 0x00, rw, 
        register_address , I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)data);
    if(err < 0){
        return err;
    }
    return 0;
}

static int i2c_wait_ack(struct i2c_adapter *a, unsigned long timeout, int writing) {
    int error = 0;
    int Status;

    struct i2c_dev_data *new_data = i2c_get_adapdata(a);
    void __iomem *pci_bar = fpga_dev.data_base_addr;

    unsigned int REG_FREQ_L;
    unsigned int REG_FREQ_H;
    unsigned int REG_CMD;
    unsigned int REG_CTRL;
    unsigned int REG_STAT;
    unsigned int REG_DATA;

    unsigned int master_bus = new_data->pca9548.master_bus;

    if (master_bus < I2C_MASTER_CH_1 || master_bus > I2C_MASTER_CH_TOTAL) {
        error = -EINVAL;
        return error;
    }

    REG_FREQ_L = I2C_MASTER_FREQ_L  + (master_bus - 1) * 0x20;
    REG_FREQ_H = I2C_MASTER_FREQ_H  + (master_bus - 1) * 0x20;
    REG_CTRL   = I2C_MASTER_CTRL    + (master_bus - 1) * 0x20;
    REG_CMD    = I2C_MASTER_CMD     + (master_bus - 1) * 0x20;
    REG_STAT   = I2C_MASTER_STATUS  + (master_bus - 1) * 0x20;
    REG_DATA   = I2C_MASTER_DATA    + (master_bus - 1) * 0x20;

    check(pci_bar + REG_STAT);
    check(pci_bar + REG_CTRL);

    dev_dbg(&a->dev,"ST:%2.2X\n", ioread8(pci_bar + REG_STAT));
    timeout = jiffies + msecs_to_jiffies(timeout);
    while (1) {
        Status = ioread8(pci_bar + REG_STAT);
        dev_dbg(&a->dev,"ST:%2.2X\n", Status);
        if (jiffies > timeout) {
            info("Status %2.2X", Status);
            info("Error Timeout");
            error = -ETIMEDOUT;
            break;
        }

        if ( Status & ( 1 << I2C_STAT_IF ) ) {
            dev_dbg(&a->dev,"  IF:%2.2X\n", Status);
            break;
        }
    }
    info("Status %2.2X", Status);
    info("STA:%x",Status);

    if (error < 0) {
        info("Status %2.2X", Status);
        return error;
    }

    // Arbitration lost
    if (Status & (1 << I2C_STAT_AL)) {
        info("Error arbitration lost");
        return -EAGAIN;
    }

    // Ack not received
    if (Status & (1 << I2C_STAT_RxACK)) {
        info( "SL No ACK");
        if (writing) {
            info("Error No ACK");
            return -EIO;
        }
    } else {
        info( "SL ACK");
    }
    return 0;
}

/* SMBUS Xfer for opencore I2C with polling */
// TODO: Change smbus_xfer to master_xfer - This will support i2c and all smbus emu functions.
static int smbus_access(struct i2c_adapter *adapter, u16 addr,
                        unsigned short flags, char rw, u8 cmd,
                        int size, union i2c_smbus_data *data)
{
    int error = 0;
    int cnt = 0;
    int bid = 0;
    struct i2c_dev_data *dev_data;
    void __iomem *pci_bar;
    unsigned int  portid, master_bus;

    unsigned int REG_FREQ_L;
    unsigned int REG_FREQ_H;
    unsigned int REG_CMD;
    unsigned int REG_CTRL;
    unsigned int REG_STAT;
    unsigned int REG_DATA;

    REG_FREQ_L = 0;
    REG_FREQ_H = 0;
    REG_CTRL   = 0;
    REG_CMD    = 0;
    REG_STAT   = 0;
    REG_DATA   = 0;

    /* Write the command register */
    dev_data = i2c_get_adapdata(adapter);
    portid = dev_data->portid;
    pci_bar = fpga_dev.data_base_addr;

#ifdef DEBUG_KERN
    printk(KERN_INFO "portid %2d|@ 0x%2.2X|f 0x%4.4X|(%d)%-5s| (%d)%-10s|CMD %2.2X "
           , portid, addr, flags, rw, rw == 1 ? "READ " : "WRITE"
           , size,                  size == 0 ? "QUICK" :
           size == 1 ? "BYTE" :
           size == 2 ? "BYTE_DATA" :
           size == 3 ? "WORD_DATA" :
           size == 4 ? "PROC_CALL" :
           size == 5 ? "BLOCK_DATA" :
           size == 8 ? "I2C_BLOCK_DATA" :  "ERROR"
           , cmd);
#endif
    /* Map the size to what the chip understands */
    switch (size) {
    case I2C_SMBUS_QUICK:
    case I2C_SMBUS_BYTE:
    case I2C_SMBUS_BYTE_DATA:
    case I2C_SMBUS_WORD_DATA:
    case I2C_SMBUS_BLOCK_DATA:
    case I2C_SMBUS_I2C_BLOCK_DATA:
        break;
    default:
        printk(KERN_INFO "Unsupported transaction %d\n", size);
        error = -EOPNOTSUPP;
        goto Done;
    }

    master_bus = dev_data->pca9548.master_bus;

    if (master_bus < I2C_MASTER_CH_1 || master_bus > I2C_MASTER_CH_TOTAL) {
        error = -EINVAL;
        goto Done;
    }

    REG_FREQ_L = I2C_MASTER_FREQ_L  + (master_bus - 1) * 0x20;
    REG_FREQ_H = I2C_MASTER_FREQ_H  + (master_bus - 1) * 0x20;
    REG_CTRL   = I2C_MASTER_CTRL    + (master_bus - 1) * 0x20;
    REG_CMD    = I2C_MASTER_CMD     + (master_bus - 1) * 0x20;
    REG_STAT   = I2C_MASTER_STATUS  + (master_bus - 1) * 0x20;
    REG_DATA   = I2C_MASTER_DATA    + (master_bus - 1) * 0x20;

    ////[S][ADDR/R]
    if (rw == I2C_SMBUS_READ &&
            (size == I2C_SMBUS_QUICK || size == I2C_SMBUS_BYTE)) {
        // sent device address with Read mode
        iowrite8( (addr << 1) | 0x01, pci_bar + REG_DATA);
    } else {
        // sent device address with Write mode
        iowrite8( (addr << 1) & 0xFE, pci_bar + REG_DATA);
    }
    iowrite8( 1 << I2C_CMD_STA | 1 << I2C_CMD_WR | 1 << I2C_CMD_IACK, pci_bar + REG_CMD);

    info( "MS Start");

    //// Wait {A}
    // + IACK
    error = i2c_wait_ack(adapter, 30, 1);
    if (error < 0) {
        info( "get error %d", error);
        dev_dbg(&adapter->dev,"START Error: %d\n", error);
        goto Done;
    }

    //// [CMD]{A}
    if (size == I2C_SMBUS_BYTE_DATA ||
            size == I2C_SMBUS_WORD_DATA ||
            size == I2C_SMBUS_BLOCK_DATA ||
            size == I2C_SMBUS_I2C_BLOCK_DATA ||
            (size == I2C_SMBUS_BYTE && rw == I2C_SMBUS_WRITE)) {

        // sent command code to data register
        iowrite8(cmd, pci_bar + REG_DATA);
        // Start the transfer
        iowrite8(1 << I2C_CMD_WR | 1 << I2C_CMD_IACK, pci_bar + REG_CMD);
        info( "MS Send CMD 0x%2.2X", cmd);

        // Wait {A}
        // IACK
        error = i2c_wait_ack(adapter, 30, 1);
        if (error < 0) {
            info( "get error %d", error);
            dev_dbg(&adapter->dev,"CMD Error: %d\n", error);
            goto Done;
        }
    }

    switch (size) {
    case I2C_SMBUS_BYTE_DATA:
        cnt = 1;  break;
    case I2C_SMBUS_WORD_DATA:
        cnt = 2;  break;
    case I2C_SMBUS_BLOCK_DATA:
    case I2C_SMBUS_I2C_BLOCK_DATA:
        /* in block data mode keep number of byte in block[0] */
        cnt = data->block[0];
        break;
    default:
        cnt = 0;  break;
    }

    // [CNT]  used only block data write
    if (size == I2C_SMBUS_BLOCK_DATA && rw == I2C_SMBUS_WRITE) {

        iowrite8(cnt, pci_bar + REG_DATA);
        //Start the transfer
        iowrite8(1 << I2C_CMD_WR | 1 << I2C_CMD_IACK, pci_bar + REG_CMD);
        info( "MS Send CNT 0x%2.2X", cnt);

        // Wait {A}
        // IACK
        error = i2c_wait_ack(adapter, 30, 1);
        if (error < 0) {
            info( "get error %d", error);
            dev_dbg(&adapter->dev,"CNT Error: %d\n", error);
            goto Done;
        }
    }

    // [DATA]{A}
    if ( rw == I2C_SMBUS_WRITE && (
                size == I2C_SMBUS_BYTE ||
                size == I2C_SMBUS_BYTE_DATA ||
                size == I2C_SMBUS_WORD_DATA ||
                size == I2C_SMBUS_BLOCK_DATA ||
                size == I2C_SMBUS_I2C_BLOCK_DATA
            )) {
        int bid = 0;
        info( "MS prepare to sent [%d bytes]", cnt);
        if (size == I2C_SMBUS_BLOCK_DATA || size == I2C_SMBUS_I2C_BLOCK_DATA) {
            bid = 1;    // block[0] is cnt;
            cnt += 1;   // offset from block[0]
        }
        for (; bid < cnt; bid++) {
            info("STA:%x", ioread8(pci_bar + REG_STAT) );
            info( "   Data > %2.2X", data->block[bid]);
            iowrite8(data->block[bid], pci_bar + REG_DATA);
            iowrite8(1 << I2C_CMD_WR | 1 << I2C_CMD_IACK, pci_bar + REG_CMD);

            // Wait {A}
            // IACK
            error = i2c_wait_ack(adapter, 30, 1);
            if (error < 0) {
                dev_dbg(&adapter->dev,"Send DATA Error: %d\n", error);
                goto Done;
            }
        }
    }

    //REPEATE START
    if ( rw == I2C_SMBUS_READ && (
                size == I2C_SMBUS_BYTE_DATA ||
                size == I2C_SMBUS_WORD_DATA ||
                size == I2C_SMBUS_BLOCK_DATA ||
                size == I2C_SMBUS_I2C_BLOCK_DATA
            )) {
        info( "MS Repeated Start");

        // sent Address with Read mode
        iowrite8( addr << 1 | 0x1 , pci_bar + REG_DATA);
        // SET START | WRITE
        iowrite8( 1 << I2C_CMD_STA | 1 << I2C_CMD_WR | 1 << I2C_CMD_IACK, pci_bar + REG_CMD);

        // Wait {A}
        error = i2c_wait_ack(adapter, 30, 1);
        if (error < 0) {
            dev_dbg(&adapter->dev,"Repeat START Error: %d\n", error);
            goto Done;
        }

    }

    if ( rw == I2C_SMBUS_READ && (
                size == I2C_SMBUS_BYTE ||
                size == I2C_SMBUS_BYTE_DATA ||
                size == I2C_SMBUS_WORD_DATA ||
                size == I2C_SMBUS_BLOCK_DATA ||
                size == I2C_SMBUS_I2C_BLOCK_DATA
            )) {

        switch (size) {
        case I2C_SMBUS_BYTE:
        case I2C_SMBUS_BYTE_DATA:
            cnt = 1;  break;
        case I2C_SMBUS_WORD_DATA:
            cnt = 2;  break;
        case I2C_SMBUS_BLOCK_DATA:
            /* will be changed after recived first data */
            cnt = 3;  break;
        case I2C_SMBUS_I2C_BLOCK_DATA:
            cnt = data->block[0];  break;
        default:
            cnt = 0;  break;
        }

        info( "MS Receive");

        for (bid = 0; bid < cnt; bid++) {

            // Start receive FSM
            if (bid == cnt - 1) {
                info( "READ NACK");
                iowrite8(1 << I2C_CMD_RD | 1 << I2C_CMD_ACK | 1 << I2C_CMD_IACK, pci_bar + REG_CMD);
            }else{

                iowrite8(1 << I2C_CMD_RD | 1 << I2C_CMD_IACK , pci_bar + REG_CMD);
            }
            
            // Wait {A}
            error = i2c_wait_ack(adapter, 30, 0);
            if (error < 0) {
                dev_dbg(&adapter->dev,"Receive DATA Error: %d\n", error);
                goto Done;
            }
            if(size == I2C_SMBUS_I2C_BLOCK_DATA){
            /* block[0] is read length */
                data->block[bid+1] = ioread8(pci_bar + REG_DATA);
                info( "DATA IN [%d] %2.2X", bid+1, data->block[bid+1]);
            }else {
                data->block[bid] = ioread8(pci_bar + REG_DATA);
                info( "DATA IN [%d] %2.2X", bid, data->block[bid]);
            }
            if (size == I2C_SMBUS_BLOCK_DATA && bid == 0) {
                cnt = data->block[0] + 1;
            }
        }
    }

Done:
    info( "MS STOP");
    // SET STOP
    iowrite8( 1 << I2C_CMD_STO | 1 << I2C_CMD_IACK, pci_bar + REG_CMD);
    // Polling for the STO to finish.
    i2c_wait_ack(adapter, 30, 0);
    check(pci_bar + REG_CTRL);
    check(pci_bar + REG_STAT);
#ifdef DEBUG_KERN
    printk(KERN_INFO "END --- Error code  %d", error);
#endif

    return error;
}

/**
 * Wrapper of smbus_access access with PCA9548 I2C switch management.
 * This function set PCA9548 switches to the proper slave channel.
 * Only one channel among switches chip is selected during communication time.
 *
 * Note: If the bus does not have any PCA9548 on it, the switch_addr must be
 * set to 0xFF, it will use normal smbus_access function.
 */
static int fpga_i2c_access(struct i2c_adapter *adapter, u16 addr,
                           unsigned short flags, char rw, u8 cmd,
                           int size, union i2c_smbus_data *data)
{
    int error = 0;
    struct i2c_dev_data *dev_data;
    unsigned char master_bus;
    unsigned char switch_addr;
    unsigned char channel;
    uint16_t prev_port = 0;
    unsigned char prev_switch;
    unsigned char prev_ch;
    int retry;

    dev_data = i2c_get_adapdata(adapter);
    master_bus = dev_data->pca9548.master_bus;
    switch_addr = dev_data->pca9548.switch_addr;
    channel = dev_data->pca9548.channel;

    // Acquire the master resource.
    mutex_lock(&fpga_i2c_master_locks[master_bus - 1]);
    prev_port = fpga_i2c_lasted_access_port[master_bus - 1];
    prev_switch = (unsigned char)(prev_port >> 8) & 0xFF;
    prev_ch = (unsigned char)(prev_port & 0xFF);

    if (switch_addr != 0xFF) {

        // Check lasted access switch address on a master
        if ( prev_switch != switch_addr && prev_switch != 0 ) {
            // reset prev_port PCA9548 chip
            retry = 3;
            while(retry--){
                error = smbus_access(adapter, (u16)(prev_switch), flags, I2C_SMBUS_WRITE, 0x00, I2C_SMBUS_BYTE, NULL);
                if(error >= 0){
                    break;
                }else{
                    dev_dbg(&adapter->dev,"Failed to deselect ch %d of 0x%x, CODE %d\n", prev_ch, prev_switch, error);
                }
                mdelay(1);
            }
            if(retry == 0)
                goto release_unlock;
            // set PCA9548 to current channel
            retry = 3;
            while(retry--){
                error = smbus_access(adapter, switch_addr, flags, I2C_SMBUS_WRITE, 1 << channel, I2C_SMBUS_BYTE, NULL);
                if(error >= 0){
                    break;
                }else{
                    dev_dbg(&adapter->dev,"Failed to deselect ch %d of 0x%x, CODE %d\n", prev_ch, prev_switch, error);
                }
                mdelay(1);
            }
            if(retry == 0)
                goto release_unlock;
            // update lasted port
            fpga_i2c_lasted_access_port[master_bus - 1] = switch_addr << 8 | channel;

        } else {
            // check if channel is also changes
            if ( prev_ch != channel || prev_switch == 0 ) {
                // set new PCA9548 at switch_addr to current
                retry = 3;
                while(retry--){
                    error = smbus_access(adapter, switch_addr, flags, I2C_SMBUS_WRITE, 1 << channel, I2C_SMBUS_BYTE, NULL);
                    if(error >= 0){
                        break;
                    }else{
                    dev_dbg(&adapter->dev,"Failed to deselect ch %d of 0x%x, CODE %d\n", prev_ch, prev_switch, error);
                }
                mdelay(1);
            }
            if(retry == 0)
                goto release_unlock;
                // update lasted port
                fpga_i2c_lasted_access_port[master_bus - 1] = switch_addr << 8 | channel;
            }
        }
    }

    // Do SMBus communication
    error = smbus_access(adapter, addr, flags, rw, cmd, size, data);
    if(error < 0){
        dev_dbg( &adapter->dev,"smbus_xfer failed (%d) @ 0x%2.2X|f 0x%4.4X|(%d)%-5s| (%d)%-10s|CMD %2.2X "
           , error, addr, flags, rw, rw == 1 ? "READ " : "WRITE"
           , size,                  size == 0 ? "QUICK" :
           size == 1 ? "BYTE" :
           size == 2 ? "BYTE_DATA" :
           size == 3 ? "WORD_DATA" :
           size == 4 ? "PROC_CALL" :
           size == 5 ? "BLOCK_DATA" :
           size == 8 ? "I2C_BLOCK_DATA" :  "ERROR"
           , cmd);
    }

release_unlock:    
    mutex_unlock(&fpga_i2c_master_locks[master_bus - 1]);
    dev_dbg(&adapter->dev,"switch ch %d of 0x%x -> ch %d of 0x%x\n", prev_ch, prev_switch, channel, switch_addr);
    return error;
}

/**
 * A callback function show available smbus functions.
 */
static u32 fpga_i2c_func(struct i2c_adapter *a)
{
    return I2C_FUNC_SMBUS_QUICK     |
           I2C_FUNC_SMBUS_BYTE      |
           I2C_FUNC_SMBUS_BYTE_DATA |
           I2C_FUNC_SMBUS_WORD_DATA |
           I2C_FUNC_SMBUS_BLOCK_DATA|
           I2C_FUNC_SMBUS_I2C_BLOCK;
}

static const struct i2c_algorithm phalanx_i2c_algorithm = {
    .smbus_xfer = fpga_i2c_access,
    .functionality  = fpga_i2c_func,
};

/**
 * Create virtual I2C bus adapter for switch devices
 * @param  pdev             platform device pointer
 * @param  portid           virtual i2c port id for switch device mapping
 * @param  bus_number_offset bus offset for virtual i2c adapter in system
 * @return                  i2c adapter.
 *
 * When bus_number_offset is -1, created adapter with dynamic bus number.
 * Otherwise create adapter at i2c bus = bus_number_offset + portid.
 */
static struct i2c_adapter * phalanx_i2c_init(struct platform_device *pdev, int portid, int bus_number_offset)
{
    int error;

    struct i2c_adapter *new_adapter;
    struct i2c_dev_data *new_data;

    new_adapter = kzalloc(sizeof(*new_adapter), GFP_KERNEL);
    if (!new_adapter) {
        printk(KERN_ALERT "Cannot alloc i2c adapter for %s", fpga_i2c_bus_dev[portid].calling_name);
        return NULL;
    }

    new_adapter->owner = THIS_MODULE;
    new_adapter->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
    new_adapter->algo  = &phalanx_i2c_algorithm;
    /* If the bus offset is -1, use dynamic bus number */
    if (bus_number_offset == -1) {
        new_adapter->nr = -1;
    } else {
        new_adapter->nr = bus_number_offset + portid;
    }

    new_data = kzalloc(sizeof(*new_data), GFP_KERNEL);
    if (!new_data) {
        printk(KERN_ALERT "Cannot alloc i2c data for %s", fpga_i2c_bus_dev[portid].calling_name);
        kzfree(new_adapter);
        return NULL;
    }

    new_data->portid = portid;
    new_data->pca9548.master_bus = fpga_i2c_bus_dev[portid].master_bus;
    new_data->pca9548.switch_addr = fpga_i2c_bus_dev[portid].switch_addr;
    new_data->pca9548.channel = fpga_i2c_bus_dev[portid].channel;
    strcpy(new_data->pca9548.calling_name, fpga_i2c_bus_dev[portid].calling_name);

    snprintf(new_adapter->name, sizeof(new_adapter->name),
             "SMBus I2C Adapter PortID: %s", new_data->pca9548.calling_name);

    i2c_set_adapdata(new_adapter, new_data);
    error = i2c_add_numbered_adapter(new_adapter);
    if (error < 0) {
        printk(KERN_ALERT "Cannot add i2c adapter %s", new_data->pca9548.calling_name);
        kzfree(new_adapter);
        kzfree(new_data);
        return NULL;
    }

    return new_adapter;
};

// I/O resource need.
static struct resource phalanx_resources[] = {
    {
        .start  = 0x10000000,
        .end    = 0x10001000,
        .flags  = IORESOURCE_MEM,
    },
};

static void phalanx_dev_release( struct device * dev)
{
    return;
}

static struct platform_device phalanx_dev = {
    .name           = DRIVER_NAME,
    .id             = -1,
    .num_resources  = ARRAY_SIZE(phalanx_resources),
    .resource       = phalanx_resources,
    .dev = {
        .release = phalanx_dev_release,
    }
};

/**
 * Board info for QSFP/SFP+ eeprom.
 * Note: Using OOM optoe as transceiver eeprom driver.
 * https://www.opencompute.org/wiki/Networking/SpecsAndDesigns#Open_Optical_Monitoring
 */
static struct i2c_board_info sff8436_eeprom_info[] = {
    { I2C_BOARD_INFO("optoe1", 0x50) }, //For QSFP w/ sff8436
    { I2C_BOARD_INFO("optoe2", 0x50) }, //For SFP+ w/ sff8472
};

static int phalanx_drv_probe(struct platform_device *pdev)
{
    struct resource *res;
    int ret = 0;
    int portid_count;
    uint8_t cpld1_version, cpld2_version, cpld3_version, cpld4_version;
    uint16_t prev_i2c_switch = 0;
    struct sff_device_data *sff_data;

    /* The device class need to be instantiated before this function called */
    BUG_ON(fpgafwclass == NULL);

    fpga_data = devm_kzalloc(&pdev->dev, sizeof(struct phalanx_fpga_data),
                             GFP_KERNEL);

    if (!fpga_data)
        return -ENOMEM;

    // Set default read address to VERSION
    fpga_data->fpga_read_addr = fpga_dev.data_base_addr + FPGA_VERSION;
    fpga_data->cpld1_read_addr = 0x00;
    fpga_data->cpld2_read_addr = 0x00;

    mutex_init(&fpga_data->fpga_lock);
    for (ret = I2C_MASTER_CH_1 ; ret <= I2C_MASTER_CH_TOTAL; ret++) {
        mutex_init(&fpga_i2c_master_locks[ret - 1]);
    }

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (unlikely(!res)) {
        printk(KERN_ERR "Specified Resource Not Available...\n");
        kzfree(fpga_data);
        return -1;
    }

    fpga = kobject_create_and_add("FPGA", &pdev->dev.kobj);
    if (!fpga) {
        kzfree(fpga_data);
        return -ENOMEM;
    }

    ret = sysfs_create_group(fpga, &fpga_attr_grp);
    if (ret != 0) {
        printk(KERN_ERR "Cannot create FPGA sysfs attributes\n");
        kobject_put(fpga);
        kzfree(fpga_data);
        return ret;
    }

    cpld1 = kobject_create_and_add("CPLD1", &pdev->dev.kobj);
    if (!cpld1) {
        sysfs_remove_group(fpga, &fpga_attr_grp);
        kobject_put(fpga);
        kzfree(fpga_data);
        return -ENOMEM;
    }
    ret = sysfs_create_group(cpld1, &cpld1_attr_grp);
    if (ret != 0) {
        printk(KERN_ERR "Cannot create CPLD1 sysfs attributes\n");
        kobject_put(cpld1);
        sysfs_remove_group(fpga, &fpga_attr_grp);
        kobject_put(fpga);
        kzfree(fpga_data);
        return ret;
    }

    cpld2 = kobject_create_and_add("CPLD2", &pdev->dev.kobj);
    if (!cpld2) {
        sysfs_remove_group(cpld1, &cpld1_attr_grp);
        kobject_put(cpld1);
        sysfs_remove_group(fpga, &fpga_attr_grp);
        kobject_put(fpga);
        kzfree(fpga_data);
        return -ENOMEM;
    }
    ret = sysfs_create_group(cpld2, &cpld2_attr_grp);
    if (ret != 0) {
        printk(KERN_ERR "Cannot create CPLD2 sysfs attributes\n");
        kobject_put(cpld2);
        sysfs_remove_group(cpld1, &cpld1_attr_grp);
        kobject_put(cpld1);
        sysfs_remove_group(fpga, &fpga_attr_grp);
        kobject_put(fpga);
        kzfree(fpga_data);
        return ret;
    }

    cpld3 = kobject_create_and_add("CPLD3", &pdev->dev.kobj);
    if (!cpld3) {
        sysfs_remove_group(cpld2, &cpld2_attr_grp);
        kobject_put(cpld2);
        sysfs_remove_group(cpld1, &cpld1_attr_grp);
        kobject_put(cpld1);
        sysfs_remove_group(fpga, &fpga_attr_grp);
        kobject_put(fpga);
        kzfree(fpga_data);
        return -ENOMEM;
    }
    ret = sysfs_create_group(cpld3, &cpld3_attr_grp);
    if (ret != 0) {
        printk(KERN_ERR "Cannot create CPLD3 sysfs attributes\n");
        kobject_put(cpld3);
        sysfs_remove_group(cpld2, &cpld2_attr_grp);
        kobject_put(cpld2);
        sysfs_remove_group(cpld1, &cpld1_attr_grp);
        kobject_put(cpld1);
        sysfs_remove_group(fpga, &fpga_attr_grp);
        kobject_put(fpga);
        kzfree(fpga_data);
        return ret;
    }

    cpld4 = kobject_create_and_add("CPLD4", &pdev->dev.kobj);
    if (!cpld4) {
        sysfs_remove_group(cpld3, &cpld3_attr_grp);
        kobject_put(cpld3);
        sysfs_remove_group(cpld2, &cpld2_attr_grp);
        kobject_put(cpld2);
        sysfs_remove_group(cpld1, &cpld1_attr_grp);
        kobject_put(cpld1);
        sysfs_remove_group(fpga, &fpga_attr_grp);
        kobject_put(fpga);
        kzfree(fpga_data);
        return -ENOMEM;
    }
    ret = sysfs_create_group(cpld4, &cpld4_attr_grp);
    if (ret != 0) {
        printk(KERN_ERR "Cannot create CPLD4 sysfs attributes\n");
        kobject_put(cpld4);
        sysfs_remove_group(cpld3, &cpld3_attr_grp);
        kobject_put(cpld3);
        sysfs_remove_group(cpld2, &cpld2_attr_grp);
        kobject_put(cpld2);
        sysfs_remove_group(cpld1, &cpld1_attr_grp);
        kobject_put(cpld1);
        sysfs_remove_group(fpga, &fpga_attr_grp);
        kobject_put(fpga);
        kzfree(fpga_data);
        return ret;
    }

    fancpld = kobject_create_and_add("FAN_CPLD", &pdev->dev.kobj);
    if (!fancpld) {
        sysfs_remove_group(cpld4, &cpld4_attr_grp);
        kobject_put(cpld4);
        sysfs_remove_group(cpld3, &cpld3_attr_grp);
        kobject_put(cpld3);
        sysfs_remove_group(cpld2, &cpld2_attr_grp);
        kobject_put(cpld2);
        sysfs_remove_group(cpld1, &cpld1_attr_grp);
        kobject_put(cpld1);
        sysfs_remove_group(fpga, &fpga_attr_grp);
        kobject_put(fpga);
        kzfree(fpga_data);
        return -ENOMEM;
    }
    if(!allow_unsafe_i2c_access)
        ret = sysfs_create_group(fancpld, &fancpld_no_attr_grp);
    else
        ret = sysfs_create_group(fancpld, &fancpld_attr_grp);
    if (ret != 0) {
        printk(KERN_ERR "Cannot create FAN_CPLD sysfs attributes\n");
        kobject_put(fancpld);
        sysfs_remove_group(cpld4, &cpld4_attr_grp);
        kobject_put(cpld4);
        sysfs_remove_group(cpld3, &cpld3_attr_grp);
        kobject_put(cpld3);
        sysfs_remove_group(cpld2, &cpld2_attr_grp);
        kobject_put(cpld2);
        sysfs_remove_group(cpld1, &cpld1_attr_grp);
        kobject_put(cpld1);
        sysfs_remove_group(fpga, &fpga_attr_grp);
        kobject_put(fpga);
        kzfree(fpga_data);
        return ret;
    }

    sff_dev = device_create(fpgafwclass, NULL, MKDEV(0, 0), NULL, "sff_device");
    if (IS_ERR(sff_dev)) {
        printk(KERN_ERR "Failed to create sff device\n");
        sysfs_remove_group(fancpld, &fancpld_attr_grp);
        kobject_put(fancpld);
        sysfs_remove_group(cpld4, &cpld4_attr_grp);
        kobject_put(cpld4);
        sysfs_remove_group(cpld3, &cpld3_attr_grp);
        kobject_put(cpld3);
        sysfs_remove_group(cpld2, &cpld2_attr_grp);
        kobject_put(cpld2);
        sysfs_remove_group(cpld1, &cpld1_attr_grp);
        kobject_put(cpld1);
        sysfs_remove_group(fpga, &fpga_attr_grp);
        kobject_put(fpga);
        kzfree(fpga_data);
        return PTR_ERR(sff_dev);
    }

    ret = sysfs_create_group(&sff_dev->kobj, &sff_led_test_grp);
    if (ret != 0) {
        printk(KERN_ERR "Cannot create SFF attributes\n");
        device_destroy(fpgafwclass, MKDEV(0, 0));
        sysfs_remove_group(fancpld, &fancpld_attr_grp);
        kobject_put(fancpld);
        sysfs_remove_group(cpld4, &cpld4_attr_grp);
        kobject_put(cpld4);
        sysfs_remove_group(cpld3, &cpld3_attr_grp);
        kobject_put(cpld3);
        sysfs_remove_group(cpld2, &cpld2_attr_grp);
        kobject_put(cpld2);
        sysfs_remove_group(cpld1, &cpld1_attr_grp);
        kobject_put(cpld1);
        sysfs_remove_group(fpga, &fpga_attr_grp);
        kobject_put(fpga);
        kzfree(fpga_data);
        return ret;
    }

    ret = sysfs_create_link(&pdev->dev.kobj, &sff_dev->kobj, "SFF");
    if (ret != 0) {
        sysfs_remove_group(&sff_dev->kobj, &sff_led_test_grp);
        device_destroy(fpgafwclass, MKDEV(0, 0));
        sysfs_remove_group(fancpld, &fancpld_attr_grp);
        kobject_put(fancpld);
        sysfs_remove_group(cpld4, &cpld4_attr_grp);
        kobject_put(cpld4);
        sysfs_remove_group(cpld3, &cpld3_attr_grp);
        kobject_put(cpld3);
        sysfs_remove_group(cpld2, &cpld2_attr_grp);
        kobject_put(cpld2);
        sysfs_remove_group(cpld1, &cpld1_attr_grp);
        kobject_put(cpld1);
        sysfs_remove_group(fpga, &fpga_attr_grp);
        kobject_put(fpga);
        kzfree(fpga_data);
        return ret;
    }

    for (portid_count = I2C_MASTER_CH_1; portid_count <= I2C_MASTER_CH_TOTAL; portid_count++){

        if(!allow_unsafe_i2c_access){
            if( portid_count < I2C_MASTER_CH_7 ||  
                portid_count == I2C_MASTER_CH_9 || portid_count == I2C_MASTER_CH_10 )
                continue;
        }
        ret = i2c_core_init(portid_count, I2C_DIV_100K, fpga_dev.data_base_addr);
        if (ret < 0) {
            dev_err(&pdev->dev, "Unable to init I2C core %d\n", portid_count);
            sysfs_remove_group(&sff_dev->kobj, &sff_led_test_grp);
            device_destroy(fpgafwclass, MKDEV(0, 0));
            sysfs_remove_group(fancpld, &fancpld_attr_grp);
            kobject_put(fancpld);
            sysfs_remove_group(cpld4, &cpld4_attr_grp);
            kobject_put(cpld4);
            sysfs_remove_group(cpld3, &cpld3_attr_grp);
            kobject_put(cpld3);
            sysfs_remove_group(cpld2, &cpld2_attr_grp);
            kobject_put(cpld2);
            sysfs_remove_group(cpld1, &cpld1_attr_grp);
            kobject_put(cpld1);
            sysfs_remove_group(fpga, &fpga_attr_grp);
            kobject_put(fpga);
            kzfree(fpga_data);
            return ret;
        }
    }

    for (portid_count = 0 ; portid_count < VIRTUAL_I2C_PORT_LENGTH ; portid_count++) {
        if(!allow_unsafe_i2c_access){
            if( portid_count >= FAN_I2C_CPLD_INDEX && portid_count < SW1_I2C_CPLD_INDEX ){
                fpga_data->i2c_adapter[portid_count] = NULL;
                continue;
            }
        }
        fpga_data->i2c_adapter[portid_count] = phalanx_i2c_init(pdev, portid_count, VIRTUAL_I2C_BUS_OFFSET);
    }

    /* Init SFF devices */
    for (portid_count = 0; portid_count < SFF_PORT_TOTAL; portid_count++) {
        struct i2c_adapter *i2c_adap = fpga_data->i2c_adapter[portid_count];
        if (i2c_adap) {
            fpga_data->sff_devices[portid_count] = phalanx_sff_init(portid_count);
            sff_data = dev_get_drvdata(fpga_data->sff_devices[portid_count]);
            BUG_ON(sff_data == NULL);
            if ( sff_data->port_type == QSFP ) {
                fpga_data->sff_i2c_clients[portid_count] = i2c_new_device(i2c_adap, &sff8436_eeprom_info[0]);
            } else {
                fpga_data->sff_i2c_clients[portid_count] = i2c_new_device(i2c_adap, &sff8436_eeprom_info[1]);
            }
            sff_data = NULL;
            sysfs_create_link(&fpga_data->sff_devices[portid_count]->kobj,
                              &fpga_data->sff_i2c_clients[portid_count]->dev.kobj,
                              "i2c");
        }
    }

    printk(KERN_INFO "Virtual I2C buses created\n");

#ifdef TEST_MODE
    return 0;
#endif
    fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00,
                    I2C_SMBUS_READ, 0x00, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&cpld1_version);
    fpga_i2c_access(fpga_data->i2c_adapter[SW1_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00,
                    I2C_SMBUS_READ, 0x00, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&cpld2_version);
    fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD1_SLAVE_ADDR, 0x00,
                    I2C_SMBUS_READ, 0x00, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&cpld3_version);
    fpga_i2c_access(fpga_data->i2c_adapter[SW2_I2C_CPLD_INDEX], CPLD2_SLAVE_ADDR, 0x00,
                    I2C_SMBUS_READ, 0x00, I2C_SMBUS_BYTE_DATA, (union i2c_smbus_data*)&cpld4_version);

    printk(KERN_INFO "Switch CPLD1 Version: %2.2x\n", cpld1_version);
    printk(KERN_INFO "Switch CPLD2 Version: %2.2x\n", cpld2_version);
    printk(KERN_INFO "Switch CPLD3 Version: %2.2x\n", cpld3_version);
    printk(KERN_INFO "Switch CPLD4 Version: %2.2x\n", cpld4_version);


    /* Init I2C buses that has PCA9548 switch device. */
    for (portid_count = 0; portid_count < VIRTUAL_I2C_PORT_LENGTH; portid_count++) {

        if(!allow_unsafe_i2c_access){
            if( portid_count >= FAN_I2C_CPLD_INDEX && portid_count < SW1_I2C_CPLD_INDEX ){
                continue;
            }
        }

        struct i2c_dev_data *dev_data;
        unsigned char master_bus;
        unsigned char switch_addr;

        dev_data = i2c_get_adapdata(fpga_data->i2c_adapter[portid_count]);
        master_bus = dev_data->pca9548.master_bus;
        switch_addr = dev_data->pca9548.switch_addr;

        if (switch_addr != 0xFF) {

            if (prev_i2c_switch != ( (master_bus << 8) | switch_addr) ) {
                // Found the bus with PCA9548, trying to clear all switch in it.
                smbus_access(fpga_data->i2c_adapter[portid_count], switch_addr, 0x00, I2C_SMBUS_WRITE, 0x00, I2C_SMBUS_BYTE, NULL);
                prev_i2c_switch = ( master_bus << 8 ) | switch_addr;
            }
        }
    }
    return 0;
}

static int phalanx_drv_remove(struct platform_device *pdev)
{
    int portid_count;
    struct sff_device_data *rem_data;
    struct i2c_dev_data *adap_data;

    for (portid_count = 0; portid_count < SFF_PORT_TOTAL; portid_count++) {
        sysfs_remove_link(&fpga_data->sff_devices[portid_count]->kobj, "i2c");
        i2c_unregister_device(fpga_data->sff_i2c_clients[portid_count]);
    }

    for (portid_count = 0 ; portid_count < VIRTUAL_I2C_PORT_LENGTH ; portid_count++) {
        if (fpga_data->i2c_adapter[portid_count] != NULL) {
            info(KERN_INFO "<%x>", fpga_data->i2c_adapter[portid_count]);
            adap_data = i2c_get_adapdata(fpga_data->i2c_adapter[portid_count]);
            i2c_del_adapter(fpga_data->i2c_adapter[portid_count]);
        }
    }

    for (portid_count = I2C_MASTER_CH_1; portid_count <= I2C_MASTER_CH_TOTAL; portid_count++){
        if(!allow_unsafe_i2c_access){
            if( portid_count < I2C_MASTER_CH_7 ||  
                portid_count == I2C_MASTER_CH_9 || portid_count == I2C_MASTER_CH_10 )
                continue;
        }
        i2c_core_deinit(portid_count, fpga_dev.data_base_addr);
    }

    for (portid_count = 0; portid_count < SFF_PORT_TOTAL; portid_count++) {
        if (fpga_data->sff_devices[portid_count] != NULL) {
            rem_data = dev_get_drvdata(fpga_data->sff_devices[portid_count]);
            device_unregister(fpga_data->sff_devices[portid_count]);
            put_device(fpga_data->sff_devices[portid_count]);
            kfree(rem_data);
        }
    }

    sysfs_remove_group(fpga, &fpga_attr_grp);
    sysfs_remove_group(cpld1, &cpld1_attr_grp);
    sysfs_remove_group(cpld2, &cpld2_attr_grp);
    sysfs_remove_group(cpld3, &cpld3_attr_grp);
    sysfs_remove_group(cpld4, &cpld4_attr_grp);
    sysfs_remove_group(fancpld, &fancpld_attr_grp);
    sysfs_remove_group(&sff_dev->kobj, &sff_led_test_grp);
    kobject_put(fpga);
    kobject_put(cpld1);
    kobject_put(cpld2);
    kobject_put(cpld3);
    kobject_put(cpld4);
    kobject_put(fancpld);
    device_destroy(fpgafwclass, MKDEV(0, 0));
    devm_kfree(&pdev->dev, fpga_data);
    return 0;
}

#ifdef TEST_MODE
#define FPGA_PCI_BAR_NUM 2
#else
#define FPGA_PCI_BAR_NUM 0
#endif



static const struct pci_device_id fpga_id_table[] = {
    {  PCI_VDEVICE(XILINX, FPGA_PCIE_DEVICE_ID) },
    {  PCI_VDEVICE(TEST, TEST_PCIE_DEVICE_ID) },
    {0, }
};

MODULE_DEVICE_TABLE(pci, fpga_id_table);

static int fpga_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    int err;
    struct device *dev = &pdev->dev;
    uint32_t fpga_version;

    if ((err = pci_enable_device(pdev))) {
        dev_err(dev, "pci_enable_device probe error %d for device %s\n",
                err, pci_name(pdev));
        return err;
    }

    if ((err = pci_request_regions(pdev, FPGA_PCI_NAME)) < 0) {
        dev_err(dev, "pci_request_regions error %d\n", err);
        goto pci_disable;
    }

    /* bar0: data mmio region */
    fpga_dev.data_mmio_start = pci_resource_start(pdev, FPGA_PCI_BAR_NUM);
    fpga_dev.data_mmio_len = pci_resource_len(pdev, FPGA_PCI_BAR_NUM);
    fpga_dev.data_base_addr = pci_iomap(pdev, FPGA_PCI_BAR_NUM, 0);
    if (!fpga_dev.data_base_addr) {
        dev_err(dev, "cannot iomap region of size %lu\n",
                (unsigned long)fpga_dev.data_mmio_len);
        goto pci_release;
    }
    dev_info(dev, "data_mmio iomap base = 0x%lx \n",
             (unsigned long)fpga_dev.data_base_addr);
    dev_info(dev, "data_mmio_start = 0x%lx data_mmio_len = %lu\n",
             (unsigned long)fpga_dev.data_mmio_start,
             (unsigned long)fpga_dev.data_mmio_len);

    printk(KERN_INFO "FPGA PCIe driver probe OK.\n");
    printk(KERN_INFO "FPGA ioremap registers of size %lu\n", (unsigned long)fpga_dev.data_mmio_len);
    printk(KERN_INFO "FPGA Virtual BAR %d at %8.8lx - %8.8lx\n", FPGA_PCI_BAR_NUM,
           (unsigned long)fpga_dev.data_base_addr,
           (unsigned long)(fpga_dev.data_base_addr + fpga_dev.data_mmio_len));
    printk(KERN_INFO "");
    fpga_version = ioread32(fpga_dev.data_base_addr);
    printk(KERN_INFO "FPGA Version : %8.8x\n", fpga_version);
    fpgafw_init();
    return 0;

pci_release:
    pci_release_regions(pdev);
pci_disable:
    pci_disable_device(pdev);
    return -EBUSY;
}

static void fpga_pci_remove(struct pci_dev *pdev)
{
    fpgafw_exit();
    pci_iounmap(pdev, fpga_dev.data_base_addr);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    printk(KERN_INFO "FPGA PCIe driver remove OK.\n");
};

static struct pci_driver pci_dev_ops = {
    .name       = FPGA_PCI_NAME,
    .probe      = fpga_pci_probe,
    .remove     = fpga_pci_remove,
    .id_table   = fpga_id_table,
};


static struct platform_driver phalanx_drv = {
    .probe  = phalanx_drv_probe,
    .remove = __exit_p(phalanx_drv_remove),
    .driver = {
        .name   = DRIVER_NAME,
    },
};

enum {
    READREG,
    WRITEREG
};

struct fpga_reg_data {
    uint32_t addr;
    uint32_t value;
};

static long fpgafw_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    int ret = 0;
    struct fpga_reg_data data;
    mutex_lock(&fpga_data->fpga_lock);

#ifdef TEST_MODE
    static  uint32_t status_reg;
#endif
    // Switch function to read and write.
    switch (cmd) {
    case READREG:
        if (copy_from_user(&data, (void __user*)arg, sizeof(data)) != 0) {
            mutex_unlock(&fpga_data->fpga_lock);
            return -EFAULT;
        }
        data.value = ioread32(fpga_dev.data_base_addr + data.addr);
        if (copy_to_user((void __user*)arg , &data, sizeof(data)) != 0) {
            mutex_unlock(&fpga_data->fpga_lock);
            return -EFAULT;
        }
#ifdef TEST_MODE
        if (data.addr == 0x1210) {
            switch (status_reg) {
            case 0x0000 : status_reg = 0x8000;
                break;

            case 0x8080 : status_reg = 0x80C0;
                break;
            case 0x80C0 : status_reg = 0x80F0;
                break;
            case 0x80F0 : status_reg = 0x80F8;
                break;

            }
            iowrite32(status_reg, fpga_dev.data_base_addr + 0x1210);
        }
#endif


        break;
    case WRITEREG:
        if (copy_from_user(&data, (void __user*)arg, sizeof(data)) != 0) {
            mutex_unlock(&fpga_data->fpga_lock);
            return -EFAULT;
        }
        iowrite32(data.value, fpga_dev.data_base_addr + data.addr);

#ifdef TEST_MODE
        if (data.addr == 0x1204) {
            status_reg = 0x8080;
            iowrite32(status_reg, fpga_dev.data_base_addr + 0x1210);
        }
#endif

        break;
    default:
        mutex_unlock(&fpga_data->fpga_lock);
        return -EINVAL;
    }
    mutex_unlock(&fpga_data->fpga_lock);
    return ret;
}


const struct file_operations fpgafw_fops = {
    .owner      = THIS_MODULE,
    .unlocked_ioctl = fpgafw_unlocked_ioctl,
};


static int fpgafw_init(void) {
    printk(KERN_INFO "Initializing the switchboard driver\n");
    // Try to dynamically allocate a major number for the device -- more difficult but worth it
    majorNumber = register_chrdev(0, DEVICE_NAME, &fpgafw_fops);
    if (majorNumber < 0) {
        printk(KERN_ALERT "Failed to register a major number\n");
        return majorNumber;
    }
    printk(KERN_INFO "Device registered correctly with major number %d\n", majorNumber);

    // Register the device class
    fpgafwclass = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(fpgafwclass)) {               // Check for error and clean up if there is
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Failed to register device class\n");
        return PTR_ERR(fpgafwclass);
    }
    printk(KERN_INFO "Device class registered correctly\n");

    // Register the device driver
    fpgafwdev = device_create(fpgafwclass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
    if (IS_ERR(fpgafwdev)) {                  // Clean up if there is an error
        class_destroy(fpgafwclass);           // Repeated code but the alternative is goto statements
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Failed to create the FW upgrade device node\n");
        return PTR_ERR(fpgafwdev);
    }
    printk(KERN_INFO "FPGA fw upgrade device node created correctly\n");
    return 0;
}

static void fpgafw_exit(void) {
    device_destroy(fpgafwclass, MKDEV(majorNumber, 0));     // remove the device
    class_unregister(fpgafwclass);                          // unregister the device class
    class_destroy(fpgafwclass);                             // remove the device class
    unregister_chrdev(majorNumber, DEVICE_NAME);            // unregister the major number
    printk(KERN_INFO "Goodbye!\n");
}

int phalanx_init(void)
{
    int rc;
    rc = pci_register_driver(&pci_dev_ops);
    if (rc)
        return rc;
    if (fpga_dev.data_base_addr == NULL) {
        pci_unregister_driver(&pci_dev_ops);
        printk(KERN_ALERT "FPGA PCIe device not found!\n");
        return -ENODEV;
    }
    platform_device_register(&phalanx_dev);
    platform_driver_register(&phalanx_drv);
    return 0;
}

void phalanx_exit(void)
{
    platform_driver_unregister(&phalanx_drv);
    platform_device_unregister(&phalanx_dev);
    pci_unregister_driver(&pci_dev_ops);
}

module_init(phalanx_init);
module_exit(phalanx_exit);

module_param(allow_unsafe_i2c_access, bool, 0400);
MODULE_PARM_DESC(allow_unsafe_i2c_access, "enable i2c busses despite potential races against BMC bus access");

MODULE_AUTHOR("Pradchaya P. <pphuchar@celestica.com>");
MODULE_DESCRIPTION("Celestica phalanx switchboard platform driver");
MODULE_VERSION(MOD_VERSION);
MODULE_LICENSE("GPL");
