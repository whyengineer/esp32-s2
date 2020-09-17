#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "string.h"

#define TAG "touch"

#define TH_INT_PIN 12
#define I2C_INST I2C_NUM_0
#define TH_ADDR  0x5d//0x5d

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define GT_CTRL_REG 	0X8040   	            //GT9147控制寄存器
#define GT_CFGS_REG 	0X8047   	            //GT9147配置起始地址寄存器
#define GT_CHECK_REG 	0X80FF   	            //GT9147校验和寄存器
#define GT_PID_REG 		0X8140   	            //GT9147产品ID寄存器
#define GT_MSW1_REG     0x814d          
#define GT_GSTID_REG 	0X814E   	            //GT9147当前检测到的触摸情况
#define GT_TP1_REG 		0X8150  	            //第一个触摸点数据地址
#define GT_TP2_REG 		0X8158		            //第二个触摸点数据地址
#define GT_TP3_REG 		0X8160		            //第三个触摸点数据地址
#define GT_TP4_REG 		0X8168		            //第四个触摸点数据地址
#define GT_TP5_REG 		0X8170		            //第五个触摸点数据地址  

const uint16_t GT9147_TPX_TBL[5]={GT_TP1_REG,GT_TP2_REG,GT_TP3_REG,GT_TP4_REG,GT_TP5_REG};

const uint8_t GT9147_CFG_TBL[]=
{ 
	0x62,0xE0,0x01,0x20,0x03,0x05,0x34,0xC0,0x01,0x08,
	0x28,0x0F,0x50,0x32,0x03,0x05,0x00,0x00,0x00,0x00,
	0x00,0x00,0x06,0x16,0x16,0x1F,0x14,0x89,0x28,0x0A,
	0x17,0x15,0x31,0x0D,0x00,0x00,0x08,0x22,0x04,0x11,
	0x00,0x00,0x00,0x00,0x00,0x03,0x82,0x08,0x08,0x00,
	0x00,0x0F,0x2C,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,
	0x9D,0x10,0x00,0x84,0x14,0x00,0x70,0x19,0x00,0x5F,
	0x20,0x00,0x55,0x27,0x00,0x54,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
	0x00,0x00,0x1A,0x18,0x16,0x14,0x12,0x10,0x0E,0x0C,
	0x0A,0x08,0x00,0x00,0x00,0x00,0x1F,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0x00,0x00,0x02,0x04,0x05,0x06,0x08,0x0A,0x0C,
	0x0E,0x1D,0x1E,0x1F,0x20,0x22,0x24,0x28,0x29,0xFF,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF6,
	0xFF,0xFF,0xFF,0xFF,0xCB,0x01,
};

static SemaphoreHandle_t thEvt;


static esp_err_t th_read(uint16_t reg_addr,uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TH_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr>>8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr&0xff, ACK_CHECK_EN);
    // i2c_master_stop(cmd);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TH_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_INST, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t th_write(uint16_t reg_addr,uint8_t *data_wr, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TH_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr>>8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr&0xff, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_INST, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    xSemaphoreGiveFromISR(thEvt,NULL);
}


// //扫描触摸屏(采用查询方式)
// //mode:0,正常扫描.
// //返回值:当前触屏状态.
// //0,触屏无触摸;1,触屏有触摸
// uint8_t GT9147_Scan(u8 mode)
// {
// 	u8 buf[4];
// 	u8 i=0;
// 	u8 res=0;
// 	u8 temp;
// 	u8 tempsta;
//  	static u8 t=0;//控制查询间隔,从而降低CPU占用率   
// 	t++;
// 	if((t%10)==0||t<10)//空闲时,每进入10次CTP_Scan函数才检测1次,从而节省CPU使用率
// 	{
// 		GT9147_RD_Reg(GT_GSTID_REG,&mode,1);	//读取触摸点的状态  
//  		if(mode&0X80&&((mode&0XF)<6))
// 		{
// 			temp=0;
// 			GT9147_WR_Reg(GT_GSTID_REG,&temp,1);//清标志 		
// 		}		
// 		if((mode&0XF)&&((mode&0XF)<6))
// 		{
// 			temp=0XFF<<(mode&0XF);	//将点的个数转换为1的位数,匹配tp_dev.sta定义 
// 			tempsta=tp_dev.sta;			//保存当前的tp_dev.sta值
// 			tp_dev.sta=(~temp)|TP_PRES_DOWN|TP_CATH_PRES; 
// 			tp_dev.x[4]=tp_dev.x[0];	//保存触点0的数据
// 			tp_dev.y[4]=tp_dev.y[0];
// 			for(i=0;i<5;i++)
// 			{
// 				if(tp_dev.sta&(1<<i))	//触摸有效?
// 				{
// 					GT9147_RD_Reg(GT9147_TPX_TBL[i],buf,4);	//读取XY坐标值
// 					if(tp_dev.touchtype&0X01)//横屏
// 					{
// 						#if 0
// 						tp_dev.y[i]=((u16)buf[1]<<8)+buf[0];
// 						tp_dev.x[i]=800-(((u16)buf[3]<<8)+buf[2]);
// 						#endif
						
// 						tp_dev.x[i]=((u16)buf[1]<<8)+buf[0];
// 						tp_dev.y[i]=((u16)buf[3]<<8)+buf[2];
						
// 						//tp_dev.x[i]=((u16)buf[3]<<8)+buf[2];
// 						//tp_dev.y[i]=((u16)buf[1]<<8)+buf[0];
						
						
// 					}else
// 					{
// 						tp_dev.x[i]=((u16)buf[1]<<8)+buf[0];
// 						tp_dev.y[i]=((u16)buf[3]<<8)+buf[2];
// 					}  
// 					printf("x[%d]:%d,y[%d]:%d\r\n",i,tp_dev.x[i],i,tp_dev.y[i]);
// 				}			
// 			} 
// 			res=1;
// 			if(tp_dev.x[0]>lcddev.width||tp_dev.y[0]>lcddev.height)//非法数据(坐标超出了)
// 			{ 
// 				if((mode&0XF)>1)		//有其他点有数据,则复第二个触点的数据到第一个触点.
// 				{
// 					tp_dev.x[0]=tp_dev.x[1];
// 					tp_dev.y[0]=tp_dev.y[1];
// 					t=0;				//触发一次,则会最少连续监测10次,从而提高命中率
// 				}else					//非法数据,则忽略此次数据(还原原来的)  
// 				{
// 					tp_dev.x[0]=tp_dev.x[4];
// 					tp_dev.y[0]=tp_dev.y[4];
// 					mode=0X80;		
// 					tp_dev.sta=tempsta;	//恢复tp_dev.sta
// 				}
// 			}else t=0;							//触发一次,则会最少连续监测10次,从而提高命中率
// 		}
// 	}
// 	if((mode&0X8F)==0X80)//无触摸点按下
// 	{ 
// 		if(tp_dev.sta&TP_PRES_DOWN)	//之前是被按下的
// 		{
// 			tp_dev.sta&=~(1<<7);	//标记按键松开
// 		}else						//之前就没有被按下
// 		{ 
// 			tp_dev.x[0]=0xffff;
// 			tp_dev.y[0]=0xffff;
// 			tp_dev.sta&=0XE0;	//清除点有效标记	
// 		}	 
// 	} 	
// 	if(t>240)t=10;//重新从10开始计数
// 	return res;
// }

static void th_task(void* arg)
{
    uint8_t th_num;
    uint8_t data;
    uint16_t point[2];
    for(;;) {
        xSemaphoreTake(thEvt,portMAX_DELAY);
        //     ESP_LOGI(TAG,"thouch!!");
        // }
        th_read(GT_GSTID_REG,&data,1);
        ESP_LOGI(TAG,"point:0x%x",data);
        if(xSemaphoreTake(thEvt,0)){
            ESP_LOGI(TAG,"thouch int!!");
        }
        if(data&0x80){
            th_num=data&0xf;
            data=0;
            th_write(GT_GSTID_REG,&data,1);
            if((th_num>0)&&(th_num<6)){
                for(int i=0;i<5;i++){
                    th_read(GT9147_TPX_TBL[i],(uint8_t*)point,4);
                    ESP_LOGI(TAG,"%d,x:%d,y:%d",i,point[0],point[1]);
                }
            }
        }
    }
}


esp_err_t th_init(){
    
    gpio_config_t io_conf;
    uint8_t id[5];
    esp_err_t err=ESP_OK;
    int i2c_master_port = I2C_INST;
    i2c_config_t conf;
    thEvt = xSemaphoreCreateBinary();
    xSemaphoreTake(thEvt,0);

    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << TH_INT_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(TH_INT_PIN, gpio_isr_handler, (void*) TH_INT_PIN);

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 11;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = 10;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 400000;
    i2c_param_config(i2c_master_port, &conf);
    err=i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    if(err!=ESP_OK){
        ESP_LOGE(TAG,"i2c init failed");
        return err;
    }
    err=th_read(GT_PID_REG,id,4);
    if(err!=ESP_OK){
        ESP_LOGE(TAG,"read id failed");
        return err;
    }
    id[4]='\0';
    ESP_LOGI(TAG,"id:%s",id);
    if(strcmp((char*)id,"9147")==0){
        id[0]=0X02;	
        th_write(GT_CTRL_REG,id,1);         //软复位GT5668
        th_read(GT_CFGS_REG,id,1);          //读取GT_CFGS_REG寄存器
        ESP_LOGI(TAG,"Default Ver:0x%x",id[0]);
        th_read(0x80FF,id,1);//读取校验和寄存器
        ESP_LOGI(TAG,"CheckSum Ver:0x%x",id[0]);
        // if(id[0]<0x60){
        //     th_write(GT_CFGS_REG,GT9147_CFG_TBL,sizeof(GT9147_CFG_TBL));
        // }
        vTaskDelay(10);
		id[0]=0X00;	 
		th_write(GT_CTRL_REG,id,1);//结束复位   
        // th_read(GT_MSW1_REG,id,1);//读取校验和寄存器
        // id[0]&=~0x3; //rise
        // th_write(GT_MSW1_REG,id,1);
        // th_write(GT_CFGS_REG,GT9147_CFG_TBL,sizeof(GT9147_CFG_TBL));
        // id[0]=0;
        // id[1]=1;	//是否写入到GT9147 FLASH?  即是否掉电保存
        // for(int i=0;i<sizeof(GT9147_CFG_TBL);i++)id[0]+=GT9147_CFG_TBL[i];//计算校验和
        // id[0]=(~id[0])+1;
        // th_write(GT_CHECK_REG,id,2);
    }
    xTaskCreate(th_task, "th_task", 2048, NULL, 5, NULL);

    return err;
}