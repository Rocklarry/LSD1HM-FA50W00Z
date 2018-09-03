#include <linux/kthread.h>
#include <linux/namei.h>
#include <linux/mount.h>
#include <linux/string.h>
#include <linux/slab.h>
#include "pixcir_i2c_ts_v36.h"

#if GTP_AUTO_UPDATE
struct st_update_msg {
    u8 force_update;
    u8 fw_flag;
    struct file *file;
    mm_segment_t old_fs;
    u32 fw_total_len;
};
static struct st_update_msg update_msg;

bool auto_update_flag = false;
u8 got_file_flag = 0; 
#define AUTO_SEARCH_BIN           0x01
#define BIN_FILE_READY            0x80

#define GUP_SEARCH_FILE_TIMES       50
#define UPDATE_FILE_PATH_1          "/system/etc/DCT_demo_20161017.pix"
#define FW_WR_LENTH 	   143
#define VERSION_ADDR   0x40
#define FW_FILE_LINE_LENTH (FW_WR_LENTH*2+2)
#define FW_VERSION_LENTH 50
#define FW_VERSION_BUF_LENTH (FW_VERSION_LENTH*2+2)
#define GTP_ADDR_LENGTH 1
//#define MY_DEBUG 1
//#define      CONFIG_OF      1 //enable dts


struct fw_line_data {
    char *buf;
    u8 is_delay;
    u32 len;
};

struct fw_buf {
    u32 total_line;
    u32 cur_line;
    u32 offset;
    struct fw_line_data ** buf;
};

struct fw_buf g_fw_buf;

extern struct i2c_client * i2c_connect_client;
//extern int work_pending;
u8 searching_file = 0;
extern unsigned char status_reg;
extern int bootloader_irq;
//extern int irq_flag;
extern int irq_num, irq_flag, work_pending;

extern void pixcir_reset(void);
extern u32 global_gpio_attb;
extern void judge_int_gpio_status(void);

s32 gup_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    s32 ret=-1;
    s32 retries = 0;
    
    while(retries < 5)
    {
        ret = i2c_master_recv(client,buf,len);
        if(ret == len)
            break;
        retries++;
    }

    return ret;
}

s32 gup_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
    s32 ret=-1;
    s32 retries = 0;

    while(retries < 5)
    {
        ret = i2c_master_send(client,buf,len);
        if (ret == len)
            break;
        retries++;
    }

    return ret;
}

u8*  check_version_valid(u8 *src,u32 len) {
	u32 i = 0;
	u32 count = 0;
	u8  *dest = NULL;
	
	if(src == NULL) {
	 GTP_ERROR("version data is error");
	 return NULL;
    }
    
     dest =  (u8 *)kzalloc(len, GFP_KERNEL);
    if(dest == NULL) {
        GTP_ERROR("kmalloc fail item->buf fail.");
        return NULL;
    }
	
	GTP_DEBUG("version before check data =%s,len=%d",src,len);
	
	for(i=0;i<len;i++) {
		if(src[i] >= '+' || src[i] <= 'z') {
			dest[i] = src[i];
			count++;
		}
	}
	
	dest[count] = '\0';
	GTP_DEBUG("version after check data =%s,count=%d",dest,count);
	
	return dest;
}

static void gup_search_file(s32 search_type)
{
    s32 i = 0;
    struct file *pfile = NULL;
    mm_segment_t old_fs;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    got_file_flag = 0x00;

   // msleep(60000); // wait system startup .2016 12 21 

    searching_file = 1;
    for (i = 0; i < GUP_SEARCH_FILE_TIMES; ++i)
    {
        if (0 == searching_file)
        {
            GTP_INFO("Force exiting file searching");
            got_file_flag = 0x00;
            set_fs(old_fs);
            return;
        }
        GTP_DEBUG("Search for %s,for fw update.(%d/%d)", UPDATE_FILE_PATH_1, i+1, GUP_SEARCH_FILE_TIMES);
        pfile = filp_open(UPDATE_FILE_PATH_1, O_RDONLY, 0);
        if (IS_ERR(pfile)){
            GTP_ERROR("open %s fw update failed!", UPDATE_FILE_PATH_1);
            msleep(3000);
            continue ;
        }
        GTP_INFO("Cfg file: %s for config update.", UPDATE_FILE_PATH_1);
        got_file_flag |= BIN_FILE_READY;
        update_msg.file = pfile;

        if (got_file_flag & BIN_FILE_READY){
            searching_file = 0;
            set_fs(old_fs);
            return;
        }
    }
    if(i >= GUP_SEARCH_FILE_TIMES)
        got_file_flag = 0x00;
    set_fs(old_fs);
    searching_file = 0;
}

static u8 gup_check_update_file(struct i2c_client *client, u8* path)
{
    mm_segment_t old_fs;
    got_file_flag = 0x00;

    if (path)
    {
        GTP_DEBUG("Update File path:%s, %d", path, (int)strlen(path));
        update_msg.file = filp_open(path, O_RDONLY, 0);

        if (IS_ERR(update_msg.file))
        {
            GTP_ERROR("Open update file(%s) error!", path);
            return FAIL;
        }
        got_file_flag = BIN_FILE_READY;
    }
    else
    {
        GTP_DEBUG("Update File name:%s",UPDATE_FILE_PATH_1);
        gup_search_file(AUTO_SEARCH_BIN);
        if(!(got_file_flag & BIN_FILE_READY)){
            GTP_ERROR("NULL file for firmware update");
            return FAIL;
        }
    }
    
    old_fs = get_fs();
    set_fs(KERNEL_DS);

    update_msg.file->f_op->llseek(update_msg.file, 0, SEEK_SET);
    update_msg.fw_total_len = update_msg.file->f_op->llseek(update_msg.file, 0, SEEK_END);

    GTP_DEBUG("Bin firmware actual size: %d(%dK)", update_msg.fw_total_len, update_msg.fw_total_len/1024);

    set_fs(old_fs);

    return SUCCESS;
}

static u8 ascii2hex(u8 a)
{
    s8 value = 0;

    //GTP_DEBUG("a=%c",a);

    if(a >= '0' && a <= '9'){
        value = a - '0';
    } else if(a >= 'A' && a <= 'F'){
        value = a - 'A' + 0x0A;
    } else if(a >= 'a' && a <= 'f'){
        value = a - 'a' + 0x0A;
    }else{
        value = 0xff;
    }
    return value;
}

#if 1
static s32 get_fw_version_from_reg(struct i2c_client *client,char *buf)
{
    char rdbuf[50] = {0};
    unsigned char  wrbuf[1] = {VERSION_ADDR & 0xff};
    u32 ret = 0;
    u8 *final_data = NULL;

    ret = gup_i2c_write(client,wrbuf,1);
    if(ret < 0 ) {
        GTP_ERROR("%s: i2c_master_send failed()",  __func__);
        return FAIL;
    }

    ret= gup_i2c_read(client,rdbuf,sizeof(rdbuf));
    if (ret != sizeof(rdbuf)) {
        GTP_ERROR("%s: i2c_master_recv failed(), ret=%d\n",  __func__, ret);
        return FAIL;
    }

	final_data = check_version_valid(rdbuf,strlen(rdbuf));
	if(final_data == NULL) {
		 return FAIL;
	}
    memcpy(buf,final_data,strlen(final_data));
    kfree(final_data);

    return SUCCESS;
}
#endif

u8 *get_fw_file_line_and_buf(struct file *file){

    u8 *pre_buf = NULL;
    u32 i=0;
    u32 line=0;
    u32 ret = 0;
    mm_segment_t old_fs ={0} ;
    u32 count = 0;
    u8 line_buf[512] ={0};

    pre_buf =(u8*) kzalloc(update_msg.fw_total_len, GFP_KERNEL);
    if (pre_buf==NULL) {
        GTP_ERROR("kmalloc fail pre_buf.");
        return NULL;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    file->f_op->llseek(file, 0, SEEK_SET);
    ret = file->f_op->read(file, (u8*)pre_buf,update_msg.fw_total_len, &file->f_pos);

    if (ret < 0)
    {
        GTP_ERROR("Read firmware in update file error.");
        set_fs(old_fs);
		kfree(pre_buf);
        return NULL;
    }

    for(i=0;i<update_msg.fw_total_len;i++) {
        if('\n' == pre_buf[i]) {
             if(line_buf[0] == '!' ||  line_buf[0] == '$')
                    line++;

            count = 0;
            memset(line_buf,0,sizeof(line_buf));

            continue;
        }

        if(pre_buf[i] == '\r')
            continue;

       line_buf[count] = pre_buf[i];
        count++;
    }

    if(line_buf[0] == '!' || line_buf[0] == '$')  //last line no line feed
         line++;

    g_fw_buf.total_line = line;
    GTP_DEBUG("fw total line = %d,real_total_len = %d",g_fw_buf.total_line,update_msg.fw_total_len);
    set_fs(old_fs);
    return pre_buf;
}

s32 fill_item_data(u8 *line_buf,u32 count,u32 line) {

    struct fw_line_data *item=NULL;

    item =(struct fw_line_data *) kzalloc(sizeof(struct fw_line_data), GFP_KERNEL);
    if (item==NULL) {
        GTP_ERROR("kmalloc fail item.");
        return -ENOMEM;
    }

    if(memcmp(line_buf,"!delay",count) == 0) {
        GTP_DEBUG("find delay,line=%d",line);
        item->is_delay = 1;
    } else {
        item->is_delay = 0;
    }

    item->buf =  (char *)kzalloc(count, GFP_KERNEL);
    if(item->buf == NULL) {
        GTP_ERROR("kmalloc fail item->buf fail.");
        kfree(item);
        return -ENOMEM;
    }

    memcpy(item->buf,line_buf,count);
    item->len = count;
    g_fw_buf .buf[line] =item;

    return SUCCESS;
}

void free_mem(struct fw_buf *fw_buf,int count) {
    int i;
    struct fw_line_data *item=NULL;
    if(fw_buf->buf) {
    for(i=0;i<count;i++) {
        if(fw_buf->buf[i]) {
            item = fw_buf->buf[i];
            kfree(item->buf);
            kfree(item);
        }
    }
     kfree(fw_buf->buf);
  }
  memset(&g_fw_buf,0,sizeof(struct fw_buf));
}

static s32 put_fw_to_buf(struct file *file ){
    u8 *pre_buf = NULL;
    u8 line_buf[512] ={0};
    u32 i = 0;
    u32 count = 0;
    u32 line = 0;
    s32 ret =0;

    pre_buf = get_fw_file_line_and_buf(file);

    if (pre_buf  == NULL)
    {
        GTP_ERROR("get fw line fail.");
        return FAIL;
    }

    g_fw_buf.buf = (struct fw_line_data**)kzalloc(g_fw_buf.total_line*sizeof(struct fw_line_data*), GFP_KERNEL);

    if (g_fw_buf.buf == NULL) {
        GTP_ERROR("kmalloc fail pre_buf.");
        kfree(pre_buf);
        return -ENOMEM;
    }


    for(i=0;i<update_msg.fw_total_len;i++) {
        if('\n' == pre_buf[i]){
            if(line_buf[0] == '!' ||  line_buf[0] == '$') {
                ret = fill_item_data(line_buf,count,line);

                if(ret == FAIL) {
                    GTP_DEBUG("fill item fail");
                    kfree(pre_buf);
                    free_mem(&g_fw_buf,i);
                    return FAIL;
                }
                line++;
            }

            count = 0;
            memset(line_buf,0,sizeof(line_buf));
            continue;
        }

        if(pre_buf[i] == '\r' )  //fw file line seed is windows format '\n\r' ,so skip '\r'
            continue;

        line_buf[count] = pre_buf[i];
        count++;

        if (i + 1 == update_msg.fw_total_len  && line < g_fw_buf.total_line  && (line_buf[0] == '!' || line_buf[0] == '$') )
        {
            ret = fill_item_data(line_buf,count,line);
            if(ret == FAIL)
            {
                GTP_DEBUG("fill item fail");
                kfree(pre_buf);
                free_mem(&g_fw_buf,i);
                return FAIL;
            }
        }
    }

    return SUCCESS;

}

#if 1
static s32 get_fw_version_from_file(char *dest_buf)

{
    u8 buf[FW_VERSION_LENTH] = {0};
    u8 pre_buf[FW_VERSION_LENTH] = {0};
    struct file *file = NULL;
    mm_segment_t old_fs;
    s32 ret = 0;
    s32 count = 0;
    u32 i;
    u8 *final_data = NULL;

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    file = update_msg.file;
    file->f_op->llseek(file, 0, SEEK_SET);
    ret = file->f_op->read(file, (u8*)pre_buf,FW_VERSION_LENTH, &file->f_pos);
    if (ret < 0)
    {
        GTP_ERROR("Read firmware head in update file error.");
        set_fs(old_fs);
        return FAIL;
    }

    GTP_DEBUG("[update_cfg]pre_buf: %s",pre_buf);
    for(i=0,count=0; i<FW_VERSION_LENTH; i++)
    {
        if ((pre_buf[i] == '"') || (pre_buf[i] == '!') || (pre_buf[i] == '\r'))
        {
            continue;
        }else if('\n' == pre_buf[i]){ //read first line end
            break;
        }

        buf[count++] = pre_buf[i];
    }

    GTP_DEBUG("[update_cfg]buf=%s,count=%d",buf,count);

	final_data = check_version_valid(buf,strlen(buf));
	if(final_data == NULL) {
		 return FAIL;
	}
    memcpy(dest_buf,final_data,strlen(final_data));
    kfree(final_data);

    set_fs(old_fs);
    // filp_close(file, NULL);
    file = NULL;

    return SUCCESS;
}
#endif

#if 1
static s8 gup_enter_update_judge(struct i2c_client *client)
{
    char fw_file_buf[FW_VERSION_LENTH] = {0};
    char reg_buf[FW_VERSION_LENTH] = {0};
    s32 ret = 0;

    ret = get_fw_version_from_file(fw_file_buf);
    if(FAIL == ret)
    {
        GTP_ERROR("[update_proc]get fw version from file fail.");
        return FAIL;
    }

    GTP_DEBUG("fw file version buf:%s,len=%d",fw_file_buf,strlen(fw_file_buf));
    msleep(9000); // wait system up.

    ret = get_fw_version_from_reg(client, reg_buf);
    if(FAIL == ret)
    {
        GTP_ERROR("get fw version from register fail,but enter update.");
        update_msg.force_update = 1;
        return SUCCESS;
	}

    GTP_DEBUG("fw reg version buf:%s,len=%d",reg_buf,strlen(reg_buf));

    if(strcmp(fw_file_buf,reg_buf) )
        update_msg.force_update = 1;
    else
        update_msg.force_update = 0;

    return SUCCESS;
}
#endif

void gtp_irq_disable(struct pixcir_i2c_ts_data *ts)
{
    unsigned long irqflags;

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (!ts->irq_is_disable)
    {
        ts->irq_is_disable = 1;
        disable_irq_nosync(ts->irq);
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

void gtp_irq_enable(struct pixcir_i2c_ts_data *ts)
{
    unsigned long irqflags = 0;

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (ts->irq_is_disable)
    {
        enable_irq(ts->irq);
        ts->irq_is_disable = 0;
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}


void gup_leave_update_mode(struct i2c_client *client)
{

 //   pixcir_reset();
#ifdef CONFIG_OF
    gpio_direction_input(global_gpio_attb);
#endif
    client->addr = SLAVE_ADDR;
    work_pending = 1;
   // GTP_DEBUG("[leave_update_mode]reset chip.");
}

s32 check_update_fw_write(struct i2c_client *client){
    unsigned char buf[4] = {0};
    s32 ret =0;

    ret = gup_i2c_read(client,buf,sizeof(buf));
    if(ret < 0) {
        GTP_ERROR("gup i2c read write 143 fail");
        return FAIL;
    }
    
    GTP_DEBUG("download check 4 bytes:%02x,%02x,%02x,%02x",buf[0],buf[1],buf[2],buf[3]);

    if ((buf[2] & 0xff) != 0) {
        GTP_ERROR("check update fw write fail,read data:%s",buf);
        return FAIL;
    }

    return SUCCESS;
}

static s32 line_ascii_2_hex(u8 *buf,u32 len,u8 *dest) {
    s32 i;
    u8 high,low;
    u8 i2c_data[FW_WR_LENTH] = {0};
    s32 num=0;

    for(i=1;i<len;i=i+2,num++) { // skip first char '$'
        high = 0;
        low = 0;

        high = ascii2hex(buf[i]);
        low = ascii2hex(buf[i+1]);

        if((high == 0xFF) || (low == 0xFF))
        {
            GTP_ERROR("[update_cfg] 0xFF ascii2hex fail.");
            return FAIL;
        }

        i2c_data[num] = (high<<4) + low;
    }

    memcpy(dest,i2c_data,sizeof(i2c_data));

    return SUCCESS;
}

s32 check_data_valid(struct fw_line_data *item){

    if(item  == NULL) {
        GTP_ERROR("buf is null");
        return FAIL;
    }

    if(item->buf[0] != '$')
    {
        GTP_DEBUG("write data is not begain with '$' ");
        return FAIL;
    }

    if((item->len-1)/2 != 143){
        GTP_DEBUG("write  data len is not 143");
        return FAIL;
    }

    return SUCCESS;
}

#ifdef MY_DEBUG
static s32 hex2ascii(u8 a)
{
    u8 value = 0;

    if(a >= 0 && a <= 9){
        value = a + 0x30;
    } else if(a >= 10 && a <= 15){
        value = a -0x0a+ 0x41;
    }else{
        GTP_DEBUG("not hex dig");
        return -1;
    }
    return value;
}

void print_per_write_data(u32 line ,u8* buf,u32 len){

    u32 i=0;
    u8 high=0;
    u8 low =0;
    u8 value_h = 0;
    u8 value_l = 0;

    char data[287] = {0};

    for(i=0;i<len;i++){

        value_h = 0;
        value_l = 0;
        high=0;
        low =0;


        high = (buf[i] & 0XF0) >> 4;
        low = buf[i] & 0x0F;

        value_h = hex2ascii(high);
        if(value_h < 0) {
            return;
        }

        value_l = hex2ascii(low);
        if(value_l < 0) {
            return;
        }

        data[2*i]= value_h;
        data[2*i+1]=value_l;
    }
    data[286] = '\0';
    mdelay(200);
    GTP_DEBUG("line=%d,len=%d,i2c_data=%s",line,len,data);
}
#endif

s32 enter_update_processing(struct i2c_client *client) {
    s32 ret = 0;
    u32 line = 0;
    struct fw_line_data *item = NULL;

    u8 i2c_data[FW_WR_LENTH] = {0};
#if 0
    u8 i;
    u8 data_buf[4][143] ={
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x04,0x01,0xE6,0x6D,0x14,0x6D,0x27,0x88,0x62,0x82,0xDB,0x3E,0x91,0xEE,0xCC,0xD7,0x87,0x81,0xC0,0x78,0xF9,0x26,0x02,0xDA,0xC9,0x09,0x1F,0xCE,0x2E,0xBB,0x5D,0x08,0x28,0x45,0x16,0x26,0xD0,0x0D,0xD5,0x6B,0x64,0xA8,0x5B,0x58,0x63,0x70,0x59,0x0B,0xFB,0x5E,0x4B,0x55,0x60,0x0A,0xA6,0x41,0x36,0xBF,0x1E,0x57,0x19,0x4F,0xB7,0x0B,0x2A,0x2F,0x8C,0x5D,0x64,0xAB,0xBC,0x41,0xD9,0xD8,0x43,0x9F,0x75,0xB2,0x91,0xA0,0xE3,0xA7,0x7D,0x38,0x7C,0xBA,0xB3,0xC9,0x1A,0x9C,0x9F,0x33,0x4D,0x04,0x58,0xCD,0x76,0x3B,0xBD,0x9F,0x2A,0xC9,0xF5,0x52,0xC5,0x9C,0xCF,0x47,0x65,0x68,0x1B,0x5F,0x93,0x9A,0x1D,0xF2,0xA5,0x63,0x1B,0x10,0x54,0xF2,0xDA,0x1B,0x1A,0x1D,0x4E,0x8F,0xE5,0x03,0xA9,0x53,0xE4,0xE4,0x2F,0x3D,0xDF,0x13,0x65,0xFD,0xF9,0x13,0x1A},
        {0x04,0x77,0xF1,0x56,0x69,0x58,0xC9,0xCA,0xE6,0x10,0x8D,0xF0,0x65,0x62,0x5B,0x3C,0x55,0x51,0xF3,0x03,0x43,0xEA,0xFA,0x26,0x59,0xD2,0x3E,0x76,0x5E,0x6C,0x1F,0x06,0x7E,0x53,0x6A,0xEC,0xBB,0x4A,0xFC,0x30,0x68,0x53,0xE0,0x64,0x57,0xBB,0x32,0xFC,0x89,0x50,0x1F,0x7A,0xFE,0x7B,0x64,0x56,0x66,0x53,0xC6,0x7B,0x6B,0x04,0x5E,0x79,0x0C,0x4C,0x7B,0x6E,0xAE,0x78,0xE9,0x5F,0x92,0xA0,0xD7,0x49,0xD7,0xEB,0x8C,0x76,0x54,0xC1,0x04,0xC6,0xA4,0x0E,0xCD,0x0C,0x66,0x12,0xC7,0xA1,0x1F,0x5A,0x43,0xC4,0x2B,0x18,0x7F,0x50,0x23,0xCD,0x0B,0xB9,0xC6,0xC6,0x00,0xE4,0x9A,0x75,0x02,0x76,0x2A,0x4E,0x4D,0x4E,0x98,0xEA,0xFC,0x4B,0xAC,0xEA,0xD4,0x8C,0x8C,0xC5,0xA3,0xB7,0xD6,0xBC,0x1D,0x86,0x0D,0xF1,0x03,0xF4,0x5E,0x6D,0xEC,0x5A,0x82,0xE7,0xA8},
        {0x04,0x5E,0x75,0xF3,0x85,0xD7,0xBD,0x08,0xD3,0x27,0xD5,0x17,0xFF,0x1A,0x56,0x4E,0x3E,0x13,0x35,0xC9,0xDB,0x4B,0x1E,0xAD,0x22,0xA4,0xA1,0x81,0xBF,0x8C,0xA1,0x7B,0x74,0xEA,0xEB,0x68,0xBE,0xE0,0x68,0x6C,0xE5,0x49,0x0C,0x23,0x8F,0x0D,0xAE,0x2C,0xBA,0xAA,0x9B,0x8F,0xA1,0x45,0x58,0x3B,0x5F,0xDF,0x52,0xBB,0x3C,0x4B,0x1F,0xE1,0x38,0xBD,0x8D,0xE0,0xBF,0x61,0x99,0x8A,0xCE,0x29,0xE3,0x9C,0x7A,0x97,0x59,0xE6,0xFE,0xA2,0x86,0x58,0x49,0x88,0xEB,0xE0,0xE2,0x65,0x34,0x4D,0xFA,0xA6,0xFA,0x83,0x62,0x04,0x86,0x54,0xA9,0x5A,0x88,0x0B,0x76,0x2B,0x44,0x19,0x1B,0x64,0x0E,0xA1,0x97,0x02,0x63,0x62,0x70,0x5B,0x5B,0x11,0x41,0x8F,0x89,0x07,0x98,0x08,0x36,0x10,0xF9,0xA2,0xE2,0xD0,0xD9,0xA8,0x57,0x6E,0x90,0xBF,0xDA,0xBB,0xC9,0x46,0x54}
    };

    for(i=0;i<4;i++) {

        ret = gup_i2c_write(client,data_buf[i],143);
        if(ret < 0) {
            GTP_ERROR("gup_i2c_write 143 data fail");
            return FAIL;
        }

        if(i == 2)
            mdelay(300);

         judge_int_gpio_status();
         //mdelay(50);
         ret = check_update_fw_write(client);
         if(ret < 0) {
             GTP_ERROR("gup_i2c_write 143 and check fail");
             return FAIL;
         }

    }

    return SUCCESS;
#endif

    memset(&g_fw_buf,0,sizeof(struct fw_buf));
    ret = put_fw_to_buf(update_msg.file);
    if(ret < 0) {
        GTP_ERROR("put_fw_to_buf fail");
        return FAIL;
    }

#if 0
    for(line=0;line<g_fw_buf.total_line;line++) {
        GTP_DEBUG("line=%d,len=%d,content=%s",line,g_fw_buf.buf[line]->len,g_fw_buf.buf[line]->buf);
    }
#endif

    for(line=0;line<g_fw_buf.total_line;line++) {

        if(line<2)   {   //skip 1 and 2 line
            GTP_DEBUG("skip line = %d",line);
            continue;
        }

        memset(i2c_data,0,sizeof(i2c_data));
        item = NULL;

        item = g_fw_buf.buf[line];

        if(item->is_delay) {
            GTP_DEBUG("read delay,will delay 300ms");
            mdelay(300);
            //mdelay(1000);
            continue;
        }else {
            if(check_data_valid(item) == FAIL) {
                GTP_DEBUG("item data is not valid");
                free_mem(&g_fw_buf,g_fw_buf.total_line);
                return FAIL;
            }

            ret =  line_ascii_2_hex(item->buf,item->len,i2c_data);

            if(ret < 0) {
                GTP_ERROR("line_ascii_2_hex fail");
                return FAIL;
            }

#ifdef MY_DEBUG
            print_per_write_data(line,i2c_data,sizeof(i2c_data));
#endif

            ret = gup_i2c_write(client,i2c_data,sizeof(i2c_data));
            if(ret < 0) {
                GTP_ERROR("gup_i2c_write 143 data fail");
                free_mem(&g_fw_buf,g_fw_buf.total_line);
                return FAIL;
            }

             judge_int_gpio_status();
             //mdelay(50);
             if((line >= 2)&&(line<(g_fw_buf.total_line-1))) {
	             ret = check_update_fw_write(client);
	             if(ret < 0) {
	                 GTP_ERROR("gup_i2c_write 143 and check fail");
                     free_mem(&g_fw_buf,g_fw_buf.total_line);
	                 return FAIL;
	            }
           }
        }
    }
    return SUCCESS;
}

void init_bt_mode(struct i2c_client *client) {
    client->addr = BOOTLOADER_ADDR;
   // status_reg = 7;

    work_pending = 0;
    bootloader_irq = 0;
    pixcir_reset();
    mdelay(10);
    //irq_flag = 1;
    //enable_irq(irq_num);
    work_pending = 2;
}

s32 init_bootload(struct i2c_client *client) {
    u8 data[4] = {0};
    s32 ret = 0;
    u8 value = 0;
//   struct pixcir_i2c_ts_data *ts = NULL;

//    ts = i2c_get_clientdata(client);
//    if(ts == NULL) {
//        GTP_ERROR("init_bootload get ts fail");
//        return FAIL;
//    }

    //pixcir_reset();
    //mdelay(10);

   // client->addr = BOOTLOADER_ADDR;
    //work_pending = 2;

    init_bt_mode(client);

    ret = gup_i2c_read(client,data,sizeof(data));
    if(ret < 0 ) {
        GTP_DEBUG("i2c read bootload 0x5d Fail");
        return FAIL;
    }

    GTP_DEBUG("data[3]=%02x",data[3]);

    value = data[3] & 0xFF;
    //mdelay(1000);

     if(value == 0x47 || value == 0x48 || value == 0x49 || value == 0x4A )
        return SUCCESS;
     else
        return FAIL;
}

s32 gup_enter_update_mode(struct i2c_client *client)
{
    s32  ret = 0;

    ret = init_bootload(client);
    if(ret == FAIL) {
        GTP_ERROR("init bootloader fail");
        return FAIL;
    }

    ret = enter_update_processing(client);
    if(ret != SUCCESS) {
        GTP_ERROR("update processing fail");
        return FAIL;
    }

    free_mem(&g_fw_buf,g_fw_buf.total_line);
    return  SUCCESS;
}

s32 gup_update_proc(void *dir)
{
    s32 ret = 0;
    s32 update_ret = FAIL;
    struct pixcir_i2c_ts_data *ts = NULL;
    u8 timeout = 0;
	int retry_times = 3;
	int i=0;

    GTP_DEBUG("%s:start",__func__);

    if(!i2c_connect_client){
        printk(KERN_ERR "%s:I2C client is NULL!\n", __func__);
        return -1;
    }

    ts = i2c_get_clientdata(i2c_connect_client);
    if(!ts){
        printk(KERN_ERR "%s:pixcir i2c data is NULL!\n", __func__);
        return -1;
    }

    if (searching_file){
        GTP_DEBUG("wait for auto update quitted completely");
        timeout = 0;
        searching_file = 0;
        while (timeout++ < 100)// wait for auto update quitted completely
        {
            msleep(100);
        }
    }

    update_msg.file = NULL;
    ret = gup_check_update_file(i2c_connect_client, (u8*)dir);     //20121211
    if(ret < 0)
    {
        GTP_ERROR("check update file fail.");
        goto file_fail;
    }

    GTP_DEBUG("gup_check_update_file OK");

    ret = gup_enter_update_judge(i2c_connect_client);
    if(FAIL == ret)
    {
        GTP_ERROR("[update_proc]Check *.bin file fail.");
        goto file_fail;
    }

    if(update_msg.force_update)
        GTP_INFO("enter update fw mode.");
    else{
        GTP_INFO("fw is exist,do not need update.");
       return SUCCESS;
    }

    //gtp_irq_disable(ts); //disable irq

	for(i=0; i<retry_times; i++){
	    ret = gup_enter_update_mode(i2c_connect_client);//update fw
	    if(SUCCESS== ret)
	    	break;
	}

	if(i >= retry_times){
		GTP_ERROR("[update_proc]enter update mode fail.");
	    update_ret = FAIL;
	    goto update_fail;
	}

    update_ret = SUCCESS;

 update_fail:
        GTP_DEBUG("[update_proc]leave update mode.");
        gup_leave_update_mode(i2c_connect_client);
        mdelay(100);
        ts->enter_update = 0;
        gtp_irq_enable(ts);

file_fail:
    if (update_msg.file && !IS_ERR(update_msg.file))
    {
        if (update_msg.old_fs)
        {
            set_fs(update_msg.old_fs);
        }
        filp_close(update_msg.file, NULL);
    }

    GTP_DEBUG("%s:end",__func__);
    auto_update_flag = false;
    return update_ret;
}

s32  gup_init_update_proc(struct pixcir_i2c_ts_data *ts)
{
    struct task_struct *thread = NULL;

    GTP_DEBUG("%s:start",__func__);
	auto_update_flag = true;
    //	thread = kthread_run(gup_update_proc, "update", "fl update");
    thread = kthread_run(gup_update_proc, (void*)NULL, "pixcir_fw_update");
    if (IS_ERR(thread))
    {
        printk(KERN_ERR "Failed to create update thread.\n");
		auto_update_flag = false;
        return -1;
    }

    GTP_DEBUG("%s:end",__func__);

    return 0;
}
#endif

