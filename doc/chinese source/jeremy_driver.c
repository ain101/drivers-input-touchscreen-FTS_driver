//http://blog.csdn.net/tigerlau225/article/details/9729493

/*
 * * Copyright (C) 2009, Notioni Corporation chenjian@notioni.com).
 * *
 * * Author: jeremy.chen
 * *
 * * This software program is licensed subject to the GNU General Public License
 * * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 * */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/i2c.h>
#include <linux/suspend.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/input/mt.h>
#include <linux/pm_runtime.h>
#include <linux/i2c/ft5306_touch.h>
#include <linux/pm_qos.h>

#include <mach/gpio.h>
#include <plat/mfp.h>

#define MAX_FINGER              (5)  /* ft5306 can only support max 5 point */

#define FT5306_LEN              (3+6*MAX_FINGER)
#define FT5X06_REG_VENDOR_ID    0xa8
#define FT6x06_REG_CHIP_ID      0xa3
#define FT5X06_REG_FW_VER       0xa6
#define FT_DELAY_DFLT           20
#define FT_NUM_RETRY            10
#define FT_VENDOR_BYD           0x59
#define FT_VENDOR_SHYUE         0xa0
#define FT_CHIPID_FT5606        0x08
#define FT_CHIPID_FT5406        0x55


//#define TP_DEBUG

#define POINT_PUTDOWN           (0)
#define POINT_PUTUP             (1)
#define POINT_CONTACT           (2)
#define POINT_INVALID           (3)
#undef dev_dbg
#define dev_dbg(dev, format, arg...) do{dev_printk(KERN_ERR, dev, format, ##arg);}while(0)

struct touch_finger {
    int pi;         /* point index */
    int ps;         /* point status */
    u16 px;         /* point x */
    u16 py;         /* point y */
};

struct ft5306_touch {
    struct input_dev *idev;
    struct input_dev *virtual_key;
    struct i2c_client *i2c;
    struct work_struct work;
    struct workqueue_struct *ts_workqueue;
    struct ft5306_touch_platform_data *data;
    struct touch_finger finger[MAX_FINGER];
    struct mutex lock;
    int power_on;
    int irq;
    struct pm_qos_request   cpufreq_qos_req_min;
};

static void ft5306_resume_events(struct work_struct *work);
static struct workqueue_struct *ft5306_resume_wq;
static DECLARE_WORK(ft5306_resume_work, ft5306_resume_events);

static struct ft5306_touch *touch;
#ifdef CONFIG_PM_RUNTIME
    static u8 ft5306_mode_cmd_sleep[2] = { 0xA5, 0x03 };
#endif
static u8 ft5306_cmd[2] = { 0x0, 0x0 };

#define FIRMWARE_UPGRADE 

#ifdef FIRMWARE_UPGRADE  
#define FTS_UPGRADE_LOOP                30
#define TPD_MAX_POINTS_2                2
#define TPD_MAX_POINTS_5                5
#define AUTO_CLB_NEED                   1
#define AUTO_CLB_NONEED                 0

struct Upgrade_Info {
        u8 CHIP_ID;
        u8 FTS_NAME[20];
        u8 TPD_MAX_POINTS;
        u8 AUTO_CLB;
    u16 delay_aa;       /*delay of write FT_UPGRADE_AA */
    u16 delay_55;       /*delay of write FT_UPGRADE_55 */
    u8 upgrade_id_1;    /*upgrade id 1 */
    u8 upgrade_id_2;    /*upgrade id 2 */
    u16 delay_readid;   /*delay of read id */
    u16 delay_earse_flash; /*delay of earse flash*/
};

  struct Upgrade_Info fts_updateinfo[] =
{
        {0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
        {0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x06, 100, 2000},
    {0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 1, 1500},
    {0x05,"FT6208",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,60, 30, 0x79, 0x05, 10, 2000},
    {0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
    {0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
    {0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
    {0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
    {0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
    {0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
};

struct Upgrade_Info fts_updateinfo_curr;


static int ft5306_touch_write(char *buf, int count);

 static struct i2c_client * this_client;
 /*-  FW file name for shenyue  ft5406 -*/
static unsigned char CTPM_FW_ID_A0_FT5406[]=
{
        #include "shenyue_ft5406_gf_hp.i"  // vendor-ic-strcuture-proj
};

static unsigned char CTPM_FW_ID_A0_FT5606[]=
{
        #include "shenyue_ft5606_gf_hp.i"   
};

 struct focal_fw_st{
        unsigned char* fw;
    unsigned          len;
    unsigned  char  ver;
};

 static struct focal_fw_st  CTPM_FW;
#endif

int ft5306_touch_read_reg(u8 reg, u8 *pval)
{
    int ret;
    int status;

    if (touch->i2c == NULL)
        return -1;
    ret = i2c_smbus_read_byte_data(touch->i2c, reg);
    if (ret >= 0) {
        *pval = ret;
        status = 0;
    } else {
        status = -EIO;
    }

    return status;
}

int ft5306_touch_write_reg(u8 reg, u8 val)
{
    int ret;
    int status;

    if (touch->i2c == NULL)
        return -1;
    ret = i2c_smbus_write_byte_data(touch->i2c, reg, val);
    if (ret == 0)
        status = 0;
    else
        status = -EIO;

    return status;
}

#ifdef FIRMWARE_UPGRADE
/***********************************************************************************************
Name    :    

Input   :   
                     

Output  :   

function    :   

***********************************************************************************************/
static int ft5x0x_i2c_txdata(char *txdata, int length)
{
    int ret;

    struct i2c_msg msg[] = {
        {
            .addr   = this_client->addr,
            .flags  = 0,
            .len    = length,
            .buf    = txdata,
        },
    };

    //msleep(1);
    ret = i2c_transfer(this_client->adapter, msg, 1);
    if (ret < 0)
        pr_err("%s i2c write error: %d\n", __func__, ret);

    return ret;
}


/***********************************************************************************************
Name    :    ft5x0x_write_reg

Input   :   addr -- address
                     para -- parameter

Output  :   

function    :   write register of ft5x0x

***********************************************************************************************/
static int ft5x0x_write_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    printk("guowenbinrt0 ft5x0x_write_reg,buf[0]=%d,buf[1]=%d\n",buf[0],buf[1]);
    ret = ft5x0x_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }
    
    return 0;
}

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char          FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int            FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL                     0x0
#define FTS_TRUE                         0x01
#define FTS_FALSE                    0x0
#define I2C_CTPM_ADDRESS         0x38//0x70


void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            udelay(1);
        }
    }
}


/*
[function]: 
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    
    ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[TSP]i2c_read_interface error\n");
        return FTS_FALSE;
    }
  
    return FTS_TRUE;
}

/*
[function]: 
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/*
[function]: 
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;    
    btPara2[in]    :parameter 2;    
    btPara3[in]    :parameter 3;    
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

/*
[function]: 
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    
    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
[function]: 
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}

static int ft5x06_i2c_read(const struct i2c_client *client, char *writebuf,
               int writelen, char *readbuf, int readlen)
{
    int ret;

    if (writelen > 0) {
        struct i2c_msg msgs[] = {
            {
                 .addr = client->addr,
                 .flags = 0,
                 .len = writelen,
                 .buf = writebuf,
             },
            {
                 .addr = client->addr,
                 .flags = I2C_M_RD,
                 .len = readlen,
                 .buf = readbuf,
             },
        };
        ret = i2c_transfer(client->adapter, msgs, 2);
        if (ret < 0)
            dev_err(&client->dev, "%s: i2c read error.\n",
                __func__);
    } else {
        struct i2c_msg msgs[] = {
            {
                 .addr = client->addr,
                 .flags = I2C_M_RD,
                 .len = readlen,
                 .buf = readbuf,
             },
        };
        ret = i2c_transfer(client->adapter, msgs, 1);
        if (ret < 0)
            dev_err(&client->dev, "%s:i2c read error.\n", __func__);
    }
    return ret;
}


#define    FTS_PACKET_LENGTH        128

static E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;
    FTS_DWRD k = 0;
    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;
    int      i_ret;
    u8 tp_fw_upgrade_cmd[2];

    
    for(k = 0;k <FTS_UPGRADE_LOOP;k ++) {
    /*********Step 1:Reset  CTPM *****/
        msleep(100);
        printk("[FTS] Step 1:Reset  CTPM\n");

        if(fts_updateinfo_curr.CHIP_ID==0x05 || fts_updateinfo_curr.CHIP_ID==0x06 ) 
            tp_fw_upgrade_cmd[0] = 0xbc;
        else
            tp_fw_upgrade_cmd[0] = 0xfc;
        tp_fw_upgrade_cmd[1] = 0xaa;
        ft5306_touch_write(tp_fw_upgrade_cmd, 2);
        msleep(fts_updateinfo_curr.delay_aa);

        if (fts_updateinfo_curr.CHIP_ID==0x05 || fts_updateinfo_curr.CHIP_ID==0x06 ) 
            tp_fw_upgrade_cmd[0] = 0xbc;
        else
            tp_fw_upgrade_cmd[0] = 0xfc; 
        tp_fw_upgrade_cmd[1] = 0x55;
        ft5306_touch_write(tp_fw_upgrade_cmd, 2);

        if(k<=15) {
             msleep(fts_updateinfo_curr.delay_55+i*3);
        }  else {
            msleep(fts_updateinfo_curr.delay_55-(i-15)*2);
           }

    /*********Step 2:Enter upgrade mode *****/

        auc_i2c_write_buf[0] = 0x55;
        auc_i2c_write_buf[1] = 0xaa;
        i = 0;
        do {
            i ++;
                i_ret = ft5306_touch_write(auc_i2c_write_buf, 2);
                msleep(10); 
        } while (i_ret <= 0 && i < 5 );

    /*********Step 3:check READ-ID***********************/        
        msleep(fts_updateinfo_curr.delay_readid);
        cmd_write(0x90,0x00,0x00,0x00,4);
        byte_read(reg_val,2);
        printk("lxh:Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);

        if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
        && reg_val[1] == fts_updateinfo_curr.upgrade_id_2) 
        {
            printk("lxh: Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
            break;
        } else {
            printk("lxh:ERR_READID \n");
          }
    }

     if (k >= FTS_UPGRADE_LOOP)
        return -ERR_READID;
  /*********Step 4:erase app*******************************/
    printk("lxh:[TSP] Step 4: erasing. \n");
    auc_i2c_write_buf[0] = 0x61;
    byte_write(auc_i2c_write_buf, 1);   /*erase app area */

    msleep(fts_updateinfo_curr.delay_earse_flash);

    /*erase panel parameter area */
    auc_i2c_write_buf[0] = 0x63;
    byte_write(auc_i2c_write_buf, 1);
    msleep(100);

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk("lxh:[TSP] Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
        
        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        msleep(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

      if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);    
        msleep(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);  
        msleep(20);
    }

    /*********Step 6: read out checksum***********************/
    i_ret = ft5306_touch_read_reg(0xcc, (u8 *) ®_val);
    if (i_ret>=0) {
        printk("hp_tp:**value stored in addr 0xcc is 0x%x**\n", reg_val[0]);
    } else {
        printk("hp_tp:**read addr 0xcc failed**\n");
       } 
        
    printk("lxh:[TSP] Step 6:  ecc read 0x%x, new firmware ecc 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    printk("Step 7: reset the new FW\n");
    auc_i2c_write_buf[0] = 0x07;
    byte_write(auc_i2c_write_buf, 1);
    msleep(300);    /*make sure CTP startup normally */

        return ERR_OK;
}

static int fts_ctpm_fw_upgrade_with_i_file(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;
    printk(KERN_ERR"%s\n",__func__);
    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW.fw;
    mdelay(300);
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf, CTPM_FW.len);
   if (i_ret != 0)
   {
       //error handling ...
       //TBD
   }
   return i_ret;
}

static int focal_ts_read(struct i2c_client *client, u8 reg, u8 *buf, int num)
{
    struct i2c_msg xfer_msg[2];

    xfer_msg[0].addr = client->addr;
    xfer_msg[0].len = 1;
    xfer_msg[0].flags = 0;
    xfer_msg[0].buf = ®

    xfer_msg[1].addr = client->addr;
    xfer_msg[1].len = num;
    xfer_msg[1].flags = I2C_M_RD;
    xfer_msg[1].buf = buf;
    return i2c_transfer(client->adapter, xfer_msg, 2);
}  

static void fts_ctpm_get_i_file_ver(unsigned char vendor_id, unsigned char chip_id)
{
 
    if (vendor_id == FT_VENDOR_SHYUE)      
    {
        if (chip_id == FT_CHIPID_FT5606)  {        
            CTPM_FW.fw  =   CTPM_FW_ID_A0_FT5606;
            CTPM_FW.len =   sizeof(CTPM_FW_ID_A0_FT5606);
        }
        else if (chip_id == FT_CHIPID_FT5406)     // ft5406
        {
            CTPM_FW.fw  =   CTPM_FW_ID_A0_FT5406;
            CTPM_FW.len =   sizeof(CTPM_FW_ID_A0_FT5406);
        }
        CTPM_FW.ver  = CTPM_FW.fw[CTPM_FW.len - 2];
    }
/*  else if(tp_ID == 0xa0)
    {
        CTPM_FW.fw  =   CTPM_FW_ID_A0;
        CTPM_FW.len =   sizeof(CTPM_FW_ID_94);
        CTPM_FW.ver  = CTPM_FW.fw[CTPM_FW.len - 2];
    }
    else
    {
        CTPM_FW.fw  =   CTPM_FW_ID_94;
        CTPM_FW.len =   sizeof(CTPM_FW_ID_94);
        CTPM_FW.ver  = CTPM_FW.fw[CTPM_FW.len - 2];
    }   */
}

static int fts_ctpm_auto_upg(struct i2c_client *client)
{
    unsigned char uc_tp_fm_ver;
    int           i_ret;
    int ret = 0;
    // unsigned char tp_ID
    u8 reg_id;
    u8 reg_addr;
    int tries;
    int err;

    ret = focal_ts_read(client,FT5X06_REG_FW_VER, &uc_tp_fm_ver, 1); //only read two fingers' data
    if (ret<0)
    {
        printk(KERN_ERR "%s: i2c_transfer failed\n", __func__);
        return -1;
    }
    printk(KERN_ERR " lxh:ft5x0x_read_fw_ver   uc_tp_fm_ver=%x\n",uc_tp_fm_ver);
    reg_addr = FT5X06_REG_VENDOR_ID;
    tries = FT_NUM_RETRY;
    do {
        err = ft5x06_i2c_read(client, ®_addr, 1, ®_id, 1);
        msleep(FT_DELAY_DFLT);
    } while ((err < 0) && (tries--));
    if (err < 0) {
        return -1;
    } else {
        printk("lxh:FT5X06_REG_VENDOR_ID is %x\n", reg_id); 
    }   
    if (reg_id == 0x5c) {
         #if defined (CONFIG_ODM_HP10T10_TP) 
           printk(KERN_DEBUG"**lxh:  forced to upgrade shenyue 10 inches fw**\n");
            fts_ctpm_get_i_file_ver(0xa0, 0x08);  // temp solution forced to upgrade fw
         #endif
    } else {
               printk(KERN_DEBUG"**lxh:   fw is ok**\n");
            fts_ctpm_get_i_file_ver(reg_id, fts_updateinfo_curr.CHIP_ID); 
    }

    if  (uc_tp_fm_ver == 0xa6 || uc_tp_fm_ver < CTPM_FW.ver)
    {
        msleep(100);
        printk(KERN_ERR "hp_tp:[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
        uc_tp_fm_ver, CTPM_FW.ver);
        i_ret = fts_ctpm_fw_upgrade_with_i_file();    
        if (i_ret == 0)
        {
            msleep(300);
            printk(KERN_ERR "lxhb:[FTS] upgrade to new version 0x%x -> 0x%x\n",uc_tp_fm_ver, CTPM_FW.ver);
        } else {
            printk(KERN_ERR "lxhc:[FTS] upgrade failed ret=%d.\n", i_ret);
        }
    }  else {
         printk("lxh: already the latest fw\n");
    }
       return 0;
}
#endif

static int ft5306_touch_read(char *buf, int count)
{
    int ret;
    ret = i2c_master_recv(touch->i2c, (char *) buf, count);
    return ret;
}

static int ft5306_touch_write(char *buf, int count)
{
    int ret;

    ret = i2c_master_send(touch->i2c, buf, count);

    return ret;
}

#ifdef TP_DEBUG
    static u8 buflog[FT5306_LEN * 5], *pbuf;
#endif

static inline int ft5306_touch_read_data(struct ft5306_touch *data)
{
    int ps, pi, i, b, ret;
    u8 buf[FT5306_LEN];
    u16 px, py;
    memset(data->finger, 0xFF, MAX_FINGER * sizeof(struct touch_finger));

    ret = ft5306_touch_read(buf, FT5306_LEN);
    if (ret < 0)
        goto out;

#ifdef TP_DEBUG
    pbuf = buflog;
    for (i = 0; i < FT5306_LEN; ++i)
        pbuf += sprintf(pbuf, "%02x ", buf[i]);
    dev_dbg(&data->i2c->dev, "RAW DATA: %s\n", buflog);
#endif

    for (i = 0; i < MAX_FINGER; ++i) {
        b = 3 + i * 6;
        px = ((u16) (buf[b + 0] & 0x0F) << 8) | (u16) buf[b + 1];
        py = ((u16) (buf[b + 2] & 0x0F) << 8) | (u16) buf[b + 3];
        ps = ((u16) (buf[b + 0] & 0xC0) >> 6);
        pi = ((u16) (buf[b + 2] & 0xF0) >> 4);

        data->finger[i].px = px;
        data->finger[i].py = py;
        data->finger[i].ps = ps;
        data->finger[i].pi = pi;
    }
out:
    return ret;
}

static void ft5306_touch_work(struct work_struct *work)
{
    struct i2c_client *client = touch->i2c;
    struct input_dev *input_dev = touch->idev;
    int status, i, ret;
    int pi, ps;
    u16 px, py, tmp;

    if (!touch->power_on)
        return;

    pm_qos_update_request_timeout(&touch->cpufreq_qos_req_min, LONG_MAX, 200000);

    ret = ft5306_touch_read_data(touch);

    for (i = 0; i < MAX_FINGER; ++i) {

    #ifdef TP_DEBUG
        dev_dbg(&client->dev,
            "REPP: i=%d pi=%d ps=0x%02x px=%d py=%d\n",
            i, touch->finger[i].pi, touch->finger[i].ps,
            touch->finger[i].px, touch->finger[i].py);
    #endif 
        ps = touch->finger[i].ps;
        if (POINT_INVALID == ps)
            continue;

        pi = touch->finger[i].pi;
        status = (POINT_PUTUP != ps);

        input_mt_slot(input_dev, pi);
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, status);  // put up event 

        if (status) {
            px = touch->finger[i].px;
            py = touch->finger[i].py;
            if (touch->data->abs_flag == 1) {
                tmp = px;
                px = py;
                py = tmp;
            } else if (touch->data->abs_flag == 2) {
                tmp = px;
                px = py;
                py = touch->data->abs_y_max - tmp;
            } else if (touch->data->abs_flag == 3) {
                tmp = px;
                px = touch->data->abs_x_max - py;
                py = tmp;
            }

        #ifdef TP_DEBUG
            dev_dbg(&client->dev, "Status is not  POINT_PUTUP X: %d Y:%d\n", px, py);
        #endif 

            input_report_abs(input_dev, ABS_MT_POSITION_X, px);
            input_report_abs(input_dev, ABS_MT_POSITION_Y, py);
            input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 16);
        }
    }

    input_sync(input_dev);
}

static irqreturn_t ft5306_touch_irq_handler(int irq, void *dev_id)
{
//  dev_dbg(&touch->i2c->dev, "ft5306_touch_irq_handler.\n");
    //schedule_work(&touch->work);
    queue_work(touch->ts_workqueue,&touch->work);
    return IRQ_HANDLED;
}

//static int index;
static ssize_t ft5306_reg_show(struct device *dev,
                   struct device_attribute *attr, char *buf)
{
    u8 reg_vendor_id;
    u8 reg_frame;
    int ret;
    int  retries = 5;
 
    do {     
        ret = ft5306_touch_read_reg(FT5X06_REG_VENDOR_ID, (u8 *) ®_vendor_id);
        msleep(50);   
        retries--; 
    } while ((retries > 0) && (ret < 0));
    if (ret >= 0) {
        printk("lxh: **register 0x%x: 0x%x **\n", FT5X06_REG_VENDOR_ID, reg_vendor_id);   
    } else {
        printk("lxh: ** read FT5X06_REG_IC_ID error **\n");   
        goto error;
    }
    
    if (reg_vendor_id == FT_VENDOR_BYD)
        ret = sprintf(buf, "%s\n", "BYD");  
    else if (reg_vendor_id == FT_VENDOR_SHYUE)
        ret = sprintf(buf, "%s\n", "SHENYUE");  
    else 
        goto error;
    return ret;
error:
      ret = sprintf(buf, "%s\n", "Get vendor infor fail");  
      return ret;
}

static ssize_t ft5306_reg_store(struct device *dev,
                struct device_attribute *attr,
                const char *buff, size_t len)
{
    int ret;
    char vol[256] = { 0 };
    u32 reg = 0, val = 0;
    int i;

    if (len > 256)
        len = 256;

    if ('w' == buff[0]) {
        memcpy(vol, buff + 2, 4);
        reg = (int) simple_strtoul(vol, NULL, 16);
        memcpy(vol, buff + 7, 4);
        val = (int) simple_strtoul(vol, NULL, 16);
        ft5306_cmd[0] = reg;
        ft5306_cmd[1] = val;
        ret = ft5306_touch_write(ft5306_cmd, 2);
        dev_info(dev, "write! reg:0x%x, val:0x%x\n", reg, val);

    } else if ('r' == buff[0]) {
        memcpy(vol, buff + 2, 4);
        reg = (int) simple_strtoul(vol, NULL, 16);
        ret = ft5306_touch_read_reg(reg, (u8 *) &val);
        dev_info(dev, "Read! reg:0x%x, val:0x%x\n", reg, val);

    } else if ('d' == buff[0]) {
        for (i = 0x00; i <= 0x3E; i++) {
            reg = i;
            ft5306_touch_read_reg(reg, (u8 *) &val);
            msleep(2);
            dev_info(dev, "Display! reg:0x%x, val:0x%x\n",
                 reg, val);
        }
    }
    return len;
}

static DEVICE_ATTR(reg_show, 0444, ft5306_reg_show, NULL);
static DEVICE_ATTR(reg_store, 0664, NULL, ft5306_reg_store);

static struct attribute *ft5306_attributes[] = {
    &dev_attr_reg_show.attr,
    &dev_attr_reg_store.attr,
    NULL
};

static const struct attribute_group ft5306_attr_group = {
    .attrs = ft5306_attributes,
};

#ifdef CONFIG_PM_RUNTIME
static void ft5306_touch_wakeup_reset(void)
{
    struct input_dev *input_dev = touch->idev;
    int i = 0, ret = 0;

    ret = ft5306_touch_read_data(touch);

    for (i = 0; i < MAX_FINGER; ++i) {
        input_mt_slot(input_dev, i);
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
    }
    input_sync(input_dev);
    return;
}

static int ft5306_runtime_suspend(struct device *dev)
{
    int ret, i = 0;
    flush_workqueue(ft5306_resume_wq);
    cancel_work_sync(&touch->work);
    flush_workqueue(touch->ts_workqueue);
//#if 0
sleep_retry:
    ret = ft5306_touch_write(ft5306_mode_cmd_sleep, 2);
    if (ret < 0) {
        if (i < 10) {
            msleep(5);
            i++;
            dev_dbg(&touch->i2c->dev,
            "ft5306_touch can't enter sleep, retry %d\n", i);
            goto sleep_retry;
        }
        dev_info(&touch->i2c->dev,
            "ft5306_touch can't enter sleep\n");
        return 0;
    } else
        dev_dbg(&touch->i2c->dev,
            "lxh:ft5306_touch enter sleep mode.\n");

    if (touch->data->power && touch->power_on) {
        touch->data->power(NULL, 0);
        mutex_lock(&touch->lock);
        touch->power_on = 0;
        mutex_unlock(&touch->lock);
    }
//#endif
    
    return 0;
}

static int ft5306_runtime_resume(struct device *dev)
{
    queue_work(ft5306_resume_wq, &ft5306_resume_work);
    return 0;
}
static void ft5306_resume_events (struct work_struct *work)
{
    if (touch->data->power && !touch->power_on)
        touch->data->power(NULL, 1);

    msleep(10);
    if (touch->data->reset)
        touch->data->reset();

    ft5306_touch_wakeup_reset();
    mutex_lock(&touch->lock);
    if (!touch->power_on)
        touch->power_on = 1;
    mutex_unlock(&touch->lock);
    return 0;
}
#endif              /* CONFIG_PM_RUNTIME */

static int __devinit
ft5306_touch_probe(struct i2c_client *client,
           const struct i2c_device_id *id)
{
    struct input_dev *input_dev;
    int maxx, maxy;
    int ret;
    u8  reg_val;
        int i; 
    struct kobject  *touch_vendor_id_kobj;  
    dev_dbg(&client->dev, "lxh:ft5306_touch.c----ft5306_touch_probe_20130801\n");

    touch = kzalloc(sizeof(struct ft5306_touch), GFP_KERNEL);
    if (touch == NULL)
        return -ENOMEM;

    touch->data = client->dev.platform_data;
    if (touch->data == NULL) {
        dev_dbg(&client->dev, "no platform data\n");
        return -EINVAL;
    }
    touch->i2c = client;
    touch->irq = client->irq;
    mutex_init(&touch->lock);
    if (touch->data->power && !touch->power_on) {
                printk("lxh:**Going to poweron**\n");
        touch->data->power(NULL, 1);
                
        }
    msleep(10);
    if (touch->data->reset)
        touch->data->reset();
    mutex_lock(&touch->lock);
    if (!touch->power_on)
        touch->power_on = 1;
    mutex_unlock(&touch->lock);
    /* register input device */
    input_dev = input_allocate_device();
    if (!input_dev) {
        dev_err(&client->dev, "Failed to allocate memory\n");
        ret = -ENOMEM;
        goto out;
    }

    touch->idev = input_dev;
    touch->idev->name = "ft5306-ts";
    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = &client->dev;

    maxx = touch->data->abs_x_max;
    maxy = touch->data->abs_y_max;

    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(EV_SYN, input_dev->evbit);
    __set_bit(ABS_MT_POSITION_X, input_dev->absbit);
    __set_bit(ABS_MT_POSITION_Y, input_dev->absbit);

    input_mt_init_slots(input_dev, MAX_FINGER);
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, maxx, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, maxy, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 16, 0, 0);

    ret = input_register_device(touch->idev);
    if (ret) {
        dev_dbg(&client->dev,
            "%s: unabled to register input device, ret = %d\n",
            __func__, ret);
        goto out_rg;
    }
    ft5306_resume_wq = create_singlethread_workqueue("ft5306_resume");
    if (ft5306_resume_wq == NULL) {
        ret = -ESRCH;
        printk("ft5306_resume_wq fail!\n");
        goto out_rg;
    }
    pm_runtime_enable(&client->dev);
    pm_runtime_get_sync(&client->dev);
    ret = ft5306_touch_read_reg(0x00, (u8 *) ®_val);      
 if (ret < 0) {
        dev_dbg(&client->dev, "ft5306 detect fail_1\n");
        touch->i2c = NULL;
        goto out_resume_wq;
    } else {
        dev_dbg(&client->dev, "ft5306 detect ok.\n");
    }     

 
#ifdef FIRMWARE_UPGRADE  
    //find chip info     
    ret= ft5306_touch_read_reg(FT6x06_REG_CHIP_ID, ®_val);
         if (ret < 0) {
        dev_dbg(&client->dev, "ft5306 detect fail_2!\n");
        touch->i2c = NULL;
        goto out_resume_wq;
    } else {
            printk(KERN_DEBUG"lxh:%s chip_id = 0x%x\n", __func__, reg_val);
    }  

    for (i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
    {
        if(reg_val==fts_updateinfo[i].CHIP_ID)
        {
            memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
            break;
        }
    }
    if (i>=sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
    {
        memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct Upgrade_Info));
    }
#endif  
     pm_runtime_put_sync_suspend(&client->dev);
/*if (ret < 0) {
        dev_dbg(&client->dev, "ft5306 detect fail!\n");
        touch->i2c = NULL;
        goto out_resume_wq;
    } else {
        dev_dbg(&client->dev, "ft5306 detect ok.\n");
    }  */

    if (touch->data->set_virtual_key)
        ret = touch->data->set_virtual_key(input_dev);
    BUG_ON(ret != 0);

#ifdef FIRMWARE_UPGRADE
        this_client = client;
     fts_ctpm_auto_upg(client); 
#endif
    ret = request_irq(touch->irq, ft5306_touch_irq_handler,
              IRQF_DISABLED | IRQF_TRIGGER_FALLING,
              "ft5306 touch", touch);
    if (ret < 0) {
        dev_info(&client->dev,
            "Request IRQ for Bigstream touch failed, return:%d\n",
            ret);
        goto out_resume_wq;
    }
    disable_irq(touch->irq);
    INIT_WORK(&touch->work, ft5306_touch_work);
    touch->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
    if (touch->ts_workqueue == NULL) {
        ret = -ESRCH;
        printk("ts_workqueue fail!\n");
        goto out_resume_wq;
    }
    touch->cpufreq_qos_req_min.name = "ft5306-ts";
    pm_qos_add_request(&touch->cpufreq_qos_req_min,
            PM_QOS_CPUFREQ_MIN,
            PM_QOS_DEFAULT_VALUE);
             /** node for providing vendor infor to app level **/
    touch_vendor_id_kobj = kobject_create_and_add("touch_vendor_id", NULL);
    if (touch_vendor_id_kobj) {
        ret = sysfs_create_group(touch_vendor_id_kobj, &ft5306_attr_group);  
        if (ret)  
            goto out_irg;
    } 

    pm_runtime_forbid(&client->dev);
    enable_irq(touch->irq);
    //touch->data->reset();    // temp solution for when power on first time tp not work. 
    return 0;

out_irg:
    free_irq(touch->irq, touch);
    cancel_work_sync(&touch->work);
    destroy_workqueue(touch->ts_workqueue);
out_resume_wq:
    pm_runtime_disable(&client->dev);
    cancel_work_sync(&ft5306_resume_work);
    destroy_workqueue(ft5306_resume_wq);
out_rg:
    input_free_device(touch->idev);
out:
    kfree(touch);
    return ret;

}

static int ft5306_touch_remove(struct i2c_client *client)
{
    pm_runtime_disable(&client->dev);
    free_irq(touch->irq, touch);
    cancel_work_sync(&ft5306_resume_work);
    destroy_workqueue(ft5306_resume_wq);
    cancel_work_sync(&touch->work);
    destroy_workqueue(touch->ts_workqueue);
    sysfs_remove_group(&client->dev.kobj, &ft5306_attr_group);
    input_unregister_device(touch->idev);
    return 0;
}

static const struct dev_pm_ops ft5306_ts_pmops = {
    SET_RUNTIME_PM_OPS(ft5306_runtime_suspend,
    ft5306_runtime_resume, NULL)
};

static const struct i2c_device_id ft5306_touch_id[] = {
    {"ft5306_touch", 0},
    {}
};

static struct i2c_driver ft5306_touch_driver = {
    .driver = {
        .name = "ft5306_touch",
        .owner = THIS_MODULE,
        .pm = &ft5306_ts_pmops,
    },
    .id_table = ft5306_touch_id,
    .probe = ft5306_touch_probe,
    .remove = ft5306_touch_remove,
};

module_i2c_driver(ft5306_touch_driver);

MODULE_DESCRIPTION("ft5306 touch Driver");
MODULE_LICENSE("GPL");
