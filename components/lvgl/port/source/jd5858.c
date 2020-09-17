#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2s_parallel.h"

void jd5858_set_index(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
    uint16_t start_pos, end_pos;
      /*!< CASET (2Ah): Column Address Set */

    /*!< Must write byte than byte */
    
    start_pos = x_start;
    end_pos = x_end;
    
    lcdp_write_cmd(0x2a00);
    lcdp_write_data_one(start_pos >> 8);
    lcdp_write_cmd(0x2a01);
    lcdp_write_data_one(start_pos & 0xFF);
    lcdp_write_cmd(0x2a02);
    lcdp_write_data_one(end_pos >> 8);
    lcdp_write_cmd(0x2a03);
    lcdp_write_data_one(end_pos & 0xFF);

       /*!< RASET (2Bh): Row Address Set */

    
    start_pos = y_start;
    end_pos = y_end;
    
    lcdp_write_cmd(0x2b00);
    lcdp_write_data_one(start_pos >> 8);
    lcdp_write_cmd(0x2b01);
    lcdp_write_data_one(start_pos & 0xFF);
    lcdp_write_cmd(0x2b02);
    lcdp_write_data_one(end_pos >> 8);
    lcdp_write_cmd(0x2b03);
    lcdp_write_data_one(end_pos & 0xFF);
    lcdp_write_cmd(0x2c00);    /*!< RAMWR (2Ch): Memory Write */
}



void jd5858_init(){
    vTaskDelay(10);
    // gpio_set_level(g_i2s_para_obj->conf.pin_rs, 0);
    // vTaskDelay(10);
    lcdp_write_cmd(0xF000);	lcdp_write_data_one(0x0055);
	lcdp_write_cmd(0xF001);	lcdp_write_data_one(0x00AA);
	lcdp_write_cmd(0xF002);	lcdp_write_data_one(0x0052);
	lcdp_write_cmd(0xF003);	lcdp_write_data_one(0x0008);
	lcdp_write_cmd(0xF004);	lcdp_write_data_one(0x0001);

	//*************AVDD Set AVDD 5.2V*************//
	lcdp_write_cmd(0xB000);	lcdp_write_data_one(0x000D);
	lcdp_write_cmd(0xB001);	lcdp_write_data_one(0x000D);
	lcdp_write_cmd(0xB002);	lcdp_write_data_one(0x000D);

	//************AVDD ratio****************//
	lcdp_write_cmd(0xB600);	lcdp_write_data_one(0x0034);
	lcdp_write_cmd(0xB601);	lcdp_write_data_one(0x0034);
	lcdp_write_cmd(0xB602);	lcdp_write_data_one(0x0034);
	 
	//************AVEE  -5.2V****************//
	lcdp_write_cmd(0xB100);	lcdp_write_data_one(0x000D);
	lcdp_write_cmd(0xB101);	lcdp_write_data_one(0x000D);
	lcdp_write_cmd(0xB102);	lcdp_write_data_one(0x000D);

	//***********AVEE ratio*************//
	lcdp_write_cmd(0xB700);	lcdp_write_data_one(0x0034);
	lcdp_write_cmd(0xB701);	lcdp_write_data_one(0x0034);
	lcdp_write_cmd(0xB702);	lcdp_write_data_one(0x0034);

	//***********VCL  -2.5V*************//
	lcdp_write_cmd(0xB200);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xB201);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xB202);	lcdp_write_data_one(0x0000);

	//**************VCL ratio*****************//
	lcdp_write_cmd(0xB800);	lcdp_write_data_one(0x0024);
	lcdp_write_cmd(0xB801);	lcdp_write_data_one(0x0024);
	lcdp_write_cmd(0xB802);	lcdp_write_data_one(0x0024);


	//*************VGH 15V  (Free pump)*********//
	lcdp_write_cmd(0xBF00);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xB300);	lcdp_write_data_one(0x000F);
	lcdp_write_cmd(0xB301);	lcdp_write_data_one(0x000F);
	lcdp_write_cmd(0xB302);	lcdp_write_data_one(0x000F);

	//*************VGH ratio*****************//
	lcdp_write_cmd(0xB900);	lcdp_write_data_one(0x0034);
	lcdp_write_cmd(0xB901);	lcdp_write_data_one(0x0034);
	lcdp_write_cmd(0xB902);	lcdp_write_data_one(0x0034);

	//***************VGL_REG -10V**************//
	lcdp_write_cmd(0xB500);	lcdp_write_data_one(0x0008);
	lcdp_write_cmd(0xB501);	lcdp_write_data_one(0x0008);
	lcdp_write_cmd(0xB502);	lcdp_write_data_one(0x0008);

	lcdp_write_cmd(0xC200);	lcdp_write_data_one(0x0003);
		 
	//*****************VGLX ratio***************//
	lcdp_write_cmd(0xBA00);	lcdp_write_data_one(0x0024);
	lcdp_write_cmd(0xBA01);	lcdp_write_data_one(0x0024);
	lcdp_write_cmd(0xBA02);	lcdp_write_data_one(0x0024);

	//*************VGMP/VGSP 4.5V/0V*************//
	lcdp_write_cmd(0xBC00);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xBC01);	lcdp_write_data_one(0x0078);
	lcdp_write_cmd(0xBC02);	lcdp_write_data_one(0x0000);

	//************VGMN/VGSN -4.5V/0V****************//
	lcdp_write_cmd(0xBD00);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xBD01);	lcdp_write_data_one(0x0078);
	lcdp_write_cmd(0xBD02);	lcdp_write_data_one(0x0000);

	//************VCOM  -1.25V****************//
	lcdp_write_cmd(0xBE00);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xBE01);	lcdp_write_data_one(0x0067);

	//************Gamma Setting******************//
	lcdp_write_cmd(0xD100);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD101);	lcdp_write_data_one(0x0032);
	lcdp_write_cmd(0xD102);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD103);	lcdp_write_data_one(0x0033);
	lcdp_write_cmd(0xD104);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD105);	lcdp_write_data_one(0x0041);
	lcdp_write_cmd(0xD106);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD107);	lcdp_write_data_one(0x005A);
	lcdp_write_cmd(0xD108);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD109);	lcdp_write_data_one(0x0076);
	lcdp_write_cmd(0xD10A);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD10B);	lcdp_write_data_one(0x00A7);
	lcdp_write_cmd(0xD10C);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD10D);	lcdp_write_data_one(0x00CF);
	lcdp_write_cmd(0xD10E);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD10F);	lcdp_write_data_one(0x0009);
	lcdp_write_cmd(0xD110);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD111);	lcdp_write_data_one(0x0036);
	lcdp_write_cmd(0xD112);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD113);	lcdp_write_data_one(0x0073);
	lcdp_write_cmd(0xD114);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD115);	lcdp_write_data_one(0x009F);
	lcdp_write_cmd(0xD116);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD117);	lcdp_write_data_one(0x00DF);
	lcdp_write_cmd(0xD118);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD119);	lcdp_write_data_one(0x0010);
	lcdp_write_cmd(0xD11A);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD11B);	lcdp_write_data_one(0x0011);
	lcdp_write_cmd(0xD11C);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD11D);	lcdp_write_data_one(0x003D);
	lcdp_write_cmd(0xD11E);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD11F);	lcdp_write_data_one(0x0069);
	lcdp_write_cmd(0xD120);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD121);	lcdp_write_data_one(0x0081);
	lcdp_write_cmd(0xD122);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD123);	lcdp_write_data_one(0x009D);
	lcdp_write_cmd(0xD124);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD125);	lcdp_write_data_one(0x00AD);
	lcdp_write_cmd(0xD126);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD127);	lcdp_write_data_one(0x00C3);
	lcdp_write_cmd(0xD128);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD129);	lcdp_write_data_one(0x00D0);
	lcdp_write_cmd(0xD12A);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD12B);	lcdp_write_data_one(0x00E2);
	lcdp_write_cmd(0xD12C);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD12D);	lcdp_write_data_one(0x00EE);
	lcdp_write_cmd(0xD12E);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD12F);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD130);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD131);	lcdp_write_data_one(0x0026);
	lcdp_write_cmd(0xD132);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD133);	lcdp_write_data_one(0x008E);
					 
	lcdp_write_cmd(0xD200);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD201);	lcdp_write_data_one(0x0032);
	lcdp_write_cmd(0xD202);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD203);	lcdp_write_data_one(0x0033);
	lcdp_write_cmd(0xD204);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD205);	lcdp_write_data_one(0x0041);
	lcdp_write_cmd(0xD206);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD207);	lcdp_write_data_one(0x005A);
	lcdp_write_cmd(0xD208);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD209);	lcdp_write_data_one(0x0076);
	lcdp_write_cmd(0xD20A);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD20B);	lcdp_write_data_one(0x00A7);
	lcdp_write_cmd(0xD20C);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD20D);	lcdp_write_data_one(0x00CF);
	lcdp_write_cmd(0xD20E);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD20F);	lcdp_write_data_one(0x0009);
	lcdp_write_cmd(0xD210);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD211);	lcdp_write_data_one(0x0036);
	lcdp_write_cmd(0xD212);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD213);	lcdp_write_data_one(0x0073);
	lcdp_write_cmd(0xD214);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD215);	lcdp_write_data_one(0x009F);
	lcdp_write_cmd(0xD216);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD217);	lcdp_write_data_one(0x00DF);
	lcdp_write_cmd(0xD218);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD219);	lcdp_write_data_one(0x0010);
	lcdp_write_cmd(0xD21A);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD21B);	lcdp_write_data_one(0x0011);
	lcdp_write_cmd(0xD21C);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD21D);	lcdp_write_data_one(0x003D);
	lcdp_write_cmd(0xD21E);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD21F);	lcdp_write_data_one(0x0069);
	lcdp_write_cmd(0xD220);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD221);	lcdp_write_data_one(0x0081);
	lcdp_write_cmd(0xD222);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD223);	lcdp_write_data_one(0x009D);
	lcdp_write_cmd(0xD224);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD225);	lcdp_write_data_one(0x00AD);
	lcdp_write_cmd(0xD226);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD227);	lcdp_write_data_one(0x00C3);
	lcdp_write_cmd(0xD228);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD229);	lcdp_write_data_one(0x00D0);
	lcdp_write_cmd(0xD22A);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD22B);	lcdp_write_data_one(0x00E2);
	lcdp_write_cmd(0xD22C);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD22D);	lcdp_write_data_one(0x00EE);
	lcdp_write_cmd(0xD22E);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD22F);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD230);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD231);	lcdp_write_data_one(0x0026);
	lcdp_write_cmd(0xD232);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD233);	lcdp_write_data_one(0x008E);

	lcdp_write_cmd(0xD300);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD301);	lcdp_write_data_one(0x0032);
	lcdp_write_cmd(0xD302);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD303);	lcdp_write_data_one(0x0033);
	lcdp_write_cmd(0xD304);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD305);	lcdp_write_data_one(0x0041);
	lcdp_write_cmd(0xD306);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD307);	lcdp_write_data_one(0x005A);
	lcdp_write_cmd(0xD308);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD309);	lcdp_write_data_one(0x0076);
	lcdp_write_cmd(0xD30A);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD30B);	lcdp_write_data_one(0x00A7);
	lcdp_write_cmd(0xD30C);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD30D);	lcdp_write_data_one(0x00CF);
	lcdp_write_cmd(0xD30E);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD30F);	lcdp_write_data_one(0x0009);
	lcdp_write_cmd(0xD310);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD311);	lcdp_write_data_one(0x0036);
	lcdp_write_cmd(0xD312);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD313);	lcdp_write_data_one(0x0073);
	lcdp_write_cmd(0xD314);	lcdp_write_data_one(0x0001); 
	lcdp_write_cmd(0xD315);	lcdp_write_data_one(0x009F);
	lcdp_write_cmd(0xD316);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD317);	lcdp_write_data_one(0x00DF);
	lcdp_write_cmd(0xD318);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD319);	lcdp_write_data_one(0x0010);
	lcdp_write_cmd(0xD31A);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD31B);	lcdp_write_data_one(0x0011);
	lcdp_write_cmd(0xD31C);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD31D);	lcdp_write_data_one(0x003D);
	lcdp_write_cmd(0xD31E);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD31F);	lcdp_write_data_one(0x0069);
	lcdp_write_cmd(0xD320);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD321);	lcdp_write_data_one(0x0081);
	lcdp_write_cmd(0xD322);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD323);	lcdp_write_data_one(0x009D);
	lcdp_write_cmd(0xD324);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD325);	lcdp_write_data_one(0x00AD);
	lcdp_write_cmd(0xD326);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD327);	lcdp_write_data_one(0x00C3);
	lcdp_write_cmd(0xD328);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD329);	lcdp_write_data_one(0x00D0);
	lcdp_write_cmd(0xD32A);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD32B);	lcdp_write_data_one(0x00E2);
	lcdp_write_cmd(0xD32C);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD32D);	lcdp_write_data_one(0x00EE);
	lcdp_write_cmd(0xD32E);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD32F);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD330);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD331);	lcdp_write_data_one(0x0026);
	lcdp_write_cmd(0xD332);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD333);	lcdp_write_data_one(0x008E);

	lcdp_write_cmd(0xD400);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD401);	lcdp_write_data_one(0x0032);
	lcdp_write_cmd(0xD402);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD403);	lcdp_write_data_one(0x0033);
	lcdp_write_cmd(0xD404);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD405);	lcdp_write_data_one(0x0041);
	lcdp_write_cmd(0xD406);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD407);	lcdp_write_data_one(0x005A);
	lcdp_write_cmd(0xD408);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD409);	lcdp_write_data_one(0x0076);
	lcdp_write_cmd(0xD40A);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD40B);	lcdp_write_data_one(0x00A7);
	lcdp_write_cmd(0xD40C);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD40D);	lcdp_write_data_one(0x00CF);
	lcdp_write_cmd(0xD40E);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD40F);	lcdp_write_data_one(0x0009);
	lcdp_write_cmd(0xD410);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD411);	lcdp_write_data_one(0x0036);
	lcdp_write_cmd(0xD412);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD413);	lcdp_write_data_one(0x0073);
	lcdp_write_cmd(0xD414);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD415);	lcdp_write_data_one(0x009F);
	lcdp_write_cmd(0xD416);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD417);	lcdp_write_data_one(0x00DF);
	lcdp_write_cmd(0xD418);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD419);	lcdp_write_data_one(0x0010);
	lcdp_write_cmd(0xD41A);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD41B);	lcdp_write_data_one(0x0011);
	lcdp_write_cmd(0xD41C);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD41D);	lcdp_write_data_one(0x003D);
	lcdp_write_cmd(0xD41E);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD41F);	lcdp_write_data_one(0x0069);
	lcdp_write_cmd(0xD420);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD421);	lcdp_write_data_one(0x0081);
	lcdp_write_cmd(0xD422);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD423);	lcdp_write_data_one(0x009D);
	lcdp_write_cmd(0xD424);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD425);	lcdp_write_data_one(0x00AD);
	lcdp_write_cmd(0xD426);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD427);	lcdp_write_data_one(0x00C3);
	lcdp_write_cmd(0xD428);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD429);	lcdp_write_data_one(0x00D0);
	lcdp_write_cmd(0xD42A);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD42B);	lcdp_write_data_one(0x00E2);
	lcdp_write_cmd(0xD42C);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD42D);	lcdp_write_data_one(0x00EE);
	lcdp_write_cmd(0xD42E);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD42F);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD430);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD431);	lcdp_write_data_one(0x0026);
	lcdp_write_cmd(0xD432);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD433);	lcdp_write_data_one(0x008E);

	lcdp_write_cmd(0xD500);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD501);	lcdp_write_data_one(0x0032);
	lcdp_write_cmd(0xD502);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD503);	lcdp_write_data_one(0x0033);
	lcdp_write_cmd(0xD504);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD505);	lcdp_write_data_one(0x0041);
	lcdp_write_cmd(0xD506);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD507);	lcdp_write_data_one(0x005A);
	lcdp_write_cmd(0xD508);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD509);	lcdp_write_data_one(0x0076);
	lcdp_write_cmd(0xD50A);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD50B);	lcdp_write_data_one(0x00A7);
	lcdp_write_cmd(0xD50C);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD50D);	lcdp_write_data_one(0x00CF);
	lcdp_write_cmd(0xD50E);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD50F);	lcdp_write_data_one(0x0009);
	lcdp_write_cmd(0xD510);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD511);	lcdp_write_data_one(0x0036);
	lcdp_write_cmd(0xD512);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD513);	lcdp_write_data_one(0x0073);
	lcdp_write_cmd(0xD514);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD515);	lcdp_write_data_one(0x009F);
	lcdp_write_cmd(0xD516);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD517);	lcdp_write_data_one(0x00DF);
	lcdp_write_cmd(0xD518);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD519);	lcdp_write_data_one(0x0010);
	lcdp_write_cmd(0xD51A);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD51B);	lcdp_write_data_one(0x0011);
	lcdp_write_cmd(0xD51C);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD51D);	lcdp_write_data_one(0x003D);
	lcdp_write_cmd(0xD51E);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD51F);	lcdp_write_data_one(0x0069);
	lcdp_write_cmd(0xD520);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD521);	lcdp_write_data_one(0x0081);
	lcdp_write_cmd(0xD522);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD523);	lcdp_write_data_one(0x009D);
	lcdp_write_cmd(0xD524);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD525);	lcdp_write_data_one(0x00AD);
	lcdp_write_cmd(0xD526);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD527);	lcdp_write_data_one(0x00C3);
	lcdp_write_cmd(0xD528);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD529);	lcdp_write_data_one(0x00D0);
	lcdp_write_cmd(0xD52A);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD52B);	lcdp_write_data_one(0x00E2);
	lcdp_write_cmd(0xD52C);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD52D);	lcdp_write_data_one(0x00EE);
	lcdp_write_cmd(0xD52E);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD52F);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD530);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD531);	lcdp_write_data_one(0x0026);
	lcdp_write_cmd(0xD532);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD533);	lcdp_write_data_one(0x008E);

	lcdp_write_cmd(0xD600);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD601);	lcdp_write_data_one(0x0032);
	lcdp_write_cmd(0xD602);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD603);	lcdp_write_data_one(0x0033);
	lcdp_write_cmd(0xD604);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD605);	lcdp_write_data_one(0x0041);
	lcdp_write_cmd(0xD606);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD607);	lcdp_write_data_one(0x005A);
	lcdp_write_cmd(0xD608);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD609);	lcdp_write_data_one(0x0076);
	lcdp_write_cmd(0xD60A);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD60B);	lcdp_write_data_one(0x00A7);
	lcdp_write_cmd(0xD60C);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xD60D);	lcdp_write_data_one(0x00CF);
	lcdp_write_cmd(0xD60E);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD60F);	lcdp_write_data_one(0x0009);
	lcdp_write_cmd(0xD610);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD611);	lcdp_write_data_one(0x0036);
	lcdp_write_cmd(0xD612);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD613);	lcdp_write_data_one(0x0073);
	lcdp_write_cmd(0xD614);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD615);	lcdp_write_data_one(0x009F);
	lcdp_write_cmd(0xD616);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD617);	lcdp_write_data_one(0x00DF);
	lcdp_write_cmd(0xD618);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD619);	lcdp_write_data_one(0x0010);
	lcdp_write_cmd(0xD61A);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD61B);	lcdp_write_data_one(0x0011);
	lcdp_write_cmd(0xD61C);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD61D);	lcdp_write_data_one(0x003D);
	lcdp_write_cmd(0xD61E);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD61F);	lcdp_write_data_one(0x0069);
	lcdp_write_cmd(0xD620);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD621);	lcdp_write_data_one(0x0081);
	lcdp_write_cmd(0xD622);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD623);	lcdp_write_data_one(0x009D);
	lcdp_write_cmd(0xD624);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD625);	lcdp_write_data_one(0x00AD);
	lcdp_write_cmd(0xD626);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD627);	lcdp_write_data_one(0x00C3);
	lcdp_write_cmd(0xD628);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD629);	lcdp_write_data_one(0x00D0);
	lcdp_write_cmd(0xD62A);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD62B);	lcdp_write_data_one(0x00E2);
	lcdp_write_cmd(0xD62C);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xD62D);	lcdp_write_data_one(0x00EE);
	lcdp_write_cmd(0xD62E);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD62F);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xD630);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD631);	lcdp_write_data_one(0x0026);
	lcdp_write_cmd(0xD632);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xD633);	lcdp_write_data_one(0x008E);


	//**************LV2 Page 0 enable*************//
	lcdp_write_cmd(0xF000);	lcdp_write_data_one(0x0055);
	lcdp_write_cmd(0xF001);	lcdp_write_data_one(0x00AA);
	lcdp_write_cmd(0xF002);	lcdp_write_data_one(0x0052);
	lcdp_write_cmd(0xF003);	lcdp_write_data_one(0x0008);
	lcdp_write_cmd(0xF004);	lcdp_write_data_one(0x0000);

	//*************480x800*********************//
	lcdp_write_cmd(0xB500);	lcdp_write_data_one(0x0050);

	//***************Display control**************//
	lcdp_write_cmd(0xB100);	lcdp_write_data_one(0x00CC);
	lcdp_write_cmd(0xB101);	lcdp_write_data_one(0x0000);

	//***************Source hold time*************//
	lcdp_write_cmd(0xB600);	lcdp_write_data_one(0x0005);

	//**************Gate EQ control***************//
	lcdp_write_cmd(0xB700);	lcdp_write_data_one(0x0070);
	lcdp_write_cmd(0xB701);	lcdp_write_data_one(0x0070);

	//*************Source EQ control (Mode 2)******//
	lcdp_write_cmd(0xB800);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xB801);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xB802);	lcdp_write_data_one(0x0003);
	lcdp_write_cmd(0xB803);	lcdp_write_data_one(0x0003);
					
	//************Inversion mode  (2-dot)***********//
	lcdp_write_cmd(0xBC00);	lcdp_write_data_one(0x0002);
	lcdp_write_cmd(0xBC01);	lcdp_write_data_one(0x0000);
	lcdp_write_cmd(0xBC02);	lcdp_write_data_one(0x0000);
					 
	//***************Frame rate***************//      
	lcdp_write_cmd(0xBD00);	lcdp_write_data_one(0x0001);
	lcdp_write_cmd(0xBD01);	lcdp_write_data_one(0x0084);
	lcdp_write_cmd(0xBD02);	lcdp_write_data_one(0x001C);  //0X1C
	lcdp_write_cmd(0xBD03);	lcdp_write_data_one(0x001C);
	lcdp_write_cmd(0xBD04);	lcdp_write_data_one(0x0000);

	//********Timing control 4H w/ 4-Delayms *******//
	lcdp_write_cmd(0xC900);	lcdp_write_data_one(0x00D0);
	lcdp_write_cmd(0xC901);	lcdp_write_data_one(0x0002); 
	lcdp_write_cmd(0xC902);	lcdp_write_data_one(0x0050);
	lcdp_write_cmd(0xC903);	lcdp_write_data_one(0x0050);
	lcdp_write_cmd(0xC904);	lcdp_write_data_one(0x0050);

	lcdp_write_cmd(0x3600);	lcdp_write_data_one(0x0060);
	lcdp_write_cmd(0x3500);	lcdp_write_data_one(0x0000);
	
	lcdp_write_cmd(0x3A00);	lcdp_write_data_one(0x0005);	//70
	 
	lcdp_write_cmd(0x1100);
	vTaskDelay(120);
	lcdp_write_cmd(0x2900);
	vTaskDelay(50); 
	lcdp_write_cmd(0x2C00);
}