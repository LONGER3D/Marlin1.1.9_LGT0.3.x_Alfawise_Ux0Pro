#include "LGT_SCR.h"
#include <stdio.h>

#ifdef LGT_MAC
	#if ENABLED(FILAMENT_RUNOUT_SENSOR)
		#include "runout.h"
	#endif
LGT_SCR LGT_LCD;
DATA Rec_Data;
DATA Send_Data;
duration_t Duration_Time;
unsigned char data_storage[DATA_SIZE];
char total_time[7];         //Total print time for each model 
uint32_t total_print_time = 0;
char printer_work_time[31];  //Total work time of printers
int gcode_id[FILE_LIST_NUM], sel_fileid =-1;
int gcode_num=0;
millis_t recovery_time=0;
uint8_t recovery_percent = 0;
float level_z_height = 0.0;
float recovery_z_height = 0.0,recovery_E_len=0.0;
float resume_x_position=0.0,resume_y_position=0.0,resume_e_position= 0.0;
bool sd_init_flag = true;
bool tartemp_flag = false;
bool LGT_is_printing = false;
bool LGT_stop_printing = false;
bool return_home = false;
#ifdef U20_Pro
	bool led_on = true;
#endif // U20_Pro
bool xy_home = false;
#ifdef U30_Pro
	bool xyz_home = false,z_home=false;
#endif
bool leveling_wait = false;
int re_count = 0;
E_MENU_TYPE menu_type= eMENU_IDLE;
PRINTER_STATUS status_type= PRINTER_SETUP;
PRINTER_KILL_STATUS kill_type = PRINTER_NORMAL;

static char fila_type = 0;  // 0 refer to PLA, 1 refer to ABS
// 0 no check, 1 check PLA, 2 check ABS, used for "no enough temp" dialog in fila [OK] return
char menu_fila_type_chk = 0;
// 0 for 10 mm, 1 for 1 mm, 2 for 0.1 mm, used for "no enough temp" dialog in move [OK] return
char menu_move_dis_chk = 0;
static char menu_measu_dis_chk = 1;	//step 1 to 1mm and step 2 to 0.1mm
static char menu_measu_step = 0;	// 0 for not start, 1 for step 1, 2 for step 2, 3 for step 3

char cmd_E[16] = { 0 };
unsigned int filament_len = 10;
unsigned int filament_temp = 200;

const float manual_feedrate_mm_m[] = MANUAL_FEEDRATE;
	void LGT_Line_To_Current(AxisEnum axis) {
	if (!planner.is_full())
		planner.buffer_line_kinematic(current_position, MMM_TO_MMS(manual_feedrate_mm_m[(int8_t)axis]), active_extruder);
	}
	 void LGT_Printer_Total_Work_Time()
	{
		Duration_Time = (print_job_timer.duration()) + recovery_time;
		total_print_time = Duration_Time.minute()+ total_print_time;
		eeprom_write_dword((uint32_t*)EEPROM_INDEX, total_print_time);
	}
	inline void LGT_Total_Time_To_String(char* buf,uint32_t time)
	{
		uint32_t h = time / 60;
		uint32_t m = time % 60;
		sprintf_P(buf, PSTR("%lu h %lu m"), h, m);
	}
LGT_SCR::LGT_SCR()
{
	memset(data_storage, 0, sizeof(data_storage));
	memset(&Rec_Data,0,sizeof(Rec_Data));
	memset(total_time, 0, sizeof(total_time));
	memset(printer_work_time, 0, sizeof(printer_work_time));
	Rec_Data.head[0]  = DW_FH_0;
	Rec_Data.head[1]  = DW_FH_1;
	Send_Data.head[0] = DW_FH_0;
	Send_Data.head[1] = DW_FH_1;
}

void LGT_SCR::LGT_Change_Filament(int fila_len)
{
	if (fila_len >= 0)
	{
		if (menu_type == eMENU_UTILI_FILA)
		{
			LGT_Change_Page(ID_DIALOG_PRINT_FILA_LOAD);
		}
		else if (menu_type == eMENU_HOME_FILA)
		{
			LGT_Change_Page(ID_DIALOG_PRINT_FILA_LOAD);
		}
	}
	else
	{
		if (menu_type == eMENU_UTILI_FILA)
		{
			LGT_Change_Page(ID_DIALOG_UTILI_FILA_UNLOAD);
		}
		else if (menu_type == eMENU_HOME_FILA)
		{
			LGT_Change_Page(ID_DIALOG_PRINT_FILA_UNLOAD);
		}
	}
	current_position[E_AXIS] = current_position[E_AXIS]+ fila_len;
	if (fila_len>=0)      //load filament
	{
		LGT_Line_To_Current(E_AXIS);
	}
	else                //unload filament
	{ 
		if (!planner.is_full())
			planner.buffer_line_kinematic(current_position, 600, 0, current_position[E_AXIS]);
	}
	planner.synchronize();
	if (menu_type == eMENU_UTILI_FILA)
	{
		LGT_Change_Page(ID_MENU_UTILI_FILA_0 + menu_fila_type_chk);
	}
	else if (menu_type == eMENU_HOME_FILA)
	{
		planner.set_e_position_mm((destination[E_CART] = current_position[E_CART] = (resume_e_position - 2)));
		if (fila_len >= 0)
		{
			LGT_Change_Page(ID_DIALOG_LOAD_FINISH);
		}
		else
		{
			LGT_Change_Page(ID_MENU_HOME_FILA_0);
		}
	}
}
void LGT_SCR::LGT_DW_Setup()
{
	if (eeprom_read_byte((const uint8_t*)(EEPROM_INDEX + 5)) != 0)
	{
		eeprom_write_dword((uint32_t*)EEPROM_INDEX, 0);
		eeprom_write_byte((uint8_t *)(EEPROM_INDEX + 5), 0);
	}
	total_print_time = eeprom_read_dword((const uint32_t*)EEPROM_INDEX);

	LGT_Send_Data_To_Screen1(ADDR_TXT_ABOUT_MODEL, MAC_MODEL);
	LGT_Send_Data_To_Screen1(ADDR_TXT_ABOUT_SIZE, MAC_SIZE);
	LGT_Send_Data_To_Screen1(ADDR_TXT_ABOUT_FW_BOARD, BOARD_FW_VER);
}
uint8_t led_times = 1, led_status=0;
int led_counter = 0;
/*************************************
FUNCTION:	The state of LED lamp
LED:	Color of LED lamp;(LED_RED¡¢LED_GREEN¡¢LED_BLUE)
**************************************/
void LGT_SCR::LED_Bright_State(uint8_t LED, uint16_t per, uint8_t mod)
{
	if (mod == 0)  //LED lights are not lighted
	{
		analogWrite(LED_RED, 0);
		analogWrite(LED_GREEN, 0);
		analogWrite(LED_BLUE, 0);
	}
	else if (mod == 1)    //Breathing lamp
	{
		if (per > 51000) per = 51000;
		if (per < 510) per = 510;
		uint8_t	led_multiple = ceil(per / 510.0);
		if (led_times < led_multiple)
		{
			led_times++;
		}
		else
		{
			if (led_status == 0)
			{
				if (led_counter >100)
				{
					led_counter += 2;
				}
				else
					led_counter++;
				led_times = 1;
				if (led_counter >= 254)
					led_status = 1;
			}
			else
			{
				led_counter--;
				led_times = 1;
				if (led_counter <= 2)
					led_status = 0;

			}
		}
		if (LED == LED_RED)
		{
			analogWrite(LED_GREEN, 0);
			analogWrite(LED_BLUE, 0);
			analogWrite(LED, led_counter);
		}
		else if (LED == LED_GREEN)
		{
			analogWrite(LED_RED, 0);
			analogWrite(LED_BLUE, 0);
			analogWrite(LED, led_counter);
		}
		else if (LED == LED_BLUE)
		{
			analogWrite(LED_RED, 0);
			analogWrite(LED_GREEN, 0);
			analogWrite(LED, led_counter);
		}
	}
	else if (mod == 2)  //Bright
	{
		if (LED == LED_RED)
		{
			analogWrite(LED_GREEN, 0);
			analogWrite(LED_BLUE, 0);
			analogWrite(LED, 255);
		}
		else if (LED == LED_GREEN)
		{
			analogWrite(LED_RED, 0);
			analogWrite(LED_BLUE, 0);
			analogWrite(LED, 255);
		}
		else if (LED == LED_BLUE)
		{
			analogWrite(LED_RED, 0);
			analogWrite(LED_GREEN, 0);
			analogWrite(LED, 255);
		}
	}
}
void LGT_SCR::LGT_Printer_Status_Light()
{
		switch (status_type)
		{
		case PRINTER_SETUP:
			LED_Bright_State(LED_GREEN, 30000, 1);
			break;
		case PRINTER_STANDBY:
			LED_Bright_State(LED_GREEN, 30000, 1);
			break;
		case PRINTER_HEAT:
			LED_Bright_State(LED_RED, 1000, 2);
			break;
		case PRINTER_PRINTING:
			LED_Bright_State(LED_BLUE, 1000, 2);
			break;
		case PRINTER_PAUSE:
			LED_Bright_State(LED_RED, 1000, 2);
			break;
		case PRINTER_PRINTING_F:
			LED_Bright_State(LED_GREEN, 1000, 2);
			break;
		default:
			 status_type = PRINTER_STANDBY;
			break;
		}
}

void LGT_SCR::LGT_Change_Page(unsigned int pageid)
{
	memset(data_storage, 0, sizeof(data_storage));
	data_storage[0] = DW_FH_0;
	data_storage[1] = DW_FH_1;
	data_storage[2] = 0x07;
	data_storage[3] = DW_CMD_VAR_W;
	data_storage[4] = 0x00;
	data_storage[5] = 0x84;
	data_storage[6] = 0x5A;
	data_storage[7] = 0x01;
	data_storage[8] = (unsigned char)(pageid >> 8) & 0xFF;
	data_storage[9] = (unsigned char)(pageid & 0x00FF);
	for (int i = 0; i < 10; i++)
		MYSERIAL1.print(data_storage[i]);
}


/*************************************
FUNCTION:	Checking sdcard and updating file list on screen
**************************************/
void LGT_SCR::LGT_SDCard_Status_Update()
{
#if  ENABLED(SDSUPPORT)&&PIN_EXISTS(SD_DETECT)
    const uint8_t sd_status = (uint8_t)IS_SD_INSERTED;
	if (!sd_status)
	{
		if (sd_init_flag ==true)
		{
			sd_init_flag = false;
			if (!card.cardOK)
			{
				card.initsd();
				delay(2);
				if (card.cardOK)
				{
					check_print_job_recovery();
					if (!check_recovery)
					{
						if (menu_type == eMENU_FILE)
						{
							if (ii_setup == (STARTUP_COUNTER + 1))
							{
								LGT_Change_Page(ID_MENU_PRINT_FILES_O);
							}
						}
					}
					else
					{
						return_home = true;
						check_recovery = false;
						enable_Z();
						LGT_LCD.LGT_Change_Page(ID_DIALOG_PRINT_RECOVERY);
					}
					LGT_Display_Filename();
				}
			}
		}
	}
	else
	{
		if (sd_init_flag == false)
		{
			if (return_home)
			{
				return_home = false;
				menu_type = eMENU_HOME;
				LGT_Change_Page(ID_MENU_HOME);
			}
			DEHILIGHT_FILE_NAME();
			sel_fileid = -1;
			uint16_t var_addr = ADDR_TXT_PRINT_FILE_ITEM_0;
			for (int i = 0; i < gcode_num; i++)   //Cleaning filename
			{
				LGT_Clean_DW_Display_Data(var_addr);
				var_addr = var_addr+LEN_FILE_NAME;
				if (i == 10 || i == 20)
					LGT_Get_MYSERIAL1_Cmd();
			}
			LGT_Clean_DW_Display_Data(ADDR_TXT_PRINT_FILE_SELECT);
			card.release();
			gcode_num = 0;
		}
		sd_init_flag = true;
	}
#endif
}

void LGT_SCR::LGT_Clean_DW_Display_Data(unsigned int addr)
{
	memset(data_storage, 0, sizeof(data_storage));
	data_storage[0] = DW_FH_0;
	data_storage[1] = DW_FH_1;
	data_storage[2] = 0x05;
	data_storage[3] = DW_CMD_VAR_W;
	data_storage[4] = (addr & 0xFF00) >> 8;
	data_storage[5] = addr;
	data_storage[6] = 0xFF;
	data_storage[7] = 0xFF;
	for (int i = 0; i < 8; i++)
		MYSERIAL1.print(data_storage[i]);
}
void LGT_SCR::LGT_Display_Filename()
{
	gcode_num = 0;
	uint16_t var_addr = ADDR_TXT_PRINT_FILE_ITEM_0;
    const uint16_t FileCnt = card.get_num_Files();  //FileCnt:Total number of files
	for (int i = (FileCnt - 1); i >= 0; i--)     //Reverse order
	//for (int i=0;i<FileCnt;i++)                
	{
		card.getfilename(i);
		if (!card.filenameIsDir)
		{
			gcode_id[gcode_num] = i;
			gcode_num++;
			LGT_MAC_Send_Filename(var_addr, i);
			var_addr = var_addr + LEN_FILE_NAME;
			if (gcode_num == 10|| gcode_num==20)
				idle();
			if (gcode_num >= FILE_LIST_NUM)
				break;
		}
	}	
}
/*************************************
FUNCTION:	Printing SD card files to DWIN_Screen
**************************************/
void LGT_SCR::LGT_MAC_Send_Filename(uint16_t Addr, uint16_t Serial_Num)
{
	memset(data_storage, 0, sizeof(data_storage));
	data_storage[0] = DW_FH_0;
	data_storage[1] = DW_FH_1;
	data_storage[2] = 0x22;
	data_storage[3] = DW_CMD_VAR_W;
	data_storage[4] = (Addr & 0xFF00) >> 8;
	data_storage[5] = Addr;
	card.getfilename(Serial_Num);
	for (int i = 0; i < 31; i++)
	{
		data_storage[6 + i] = card.longFilename[i];
	}
	for (int i = 0; i <37; i++)
	{
		MYSERIAL1.print(data_storage[i]);
		delayMicroseconds(1);
	}
}
void LGT_SCR::LGT_Print_Cause_Of_Kill()
{
	switch (kill_type)
	{
	case E_TEMP_KILL:
		LGT_Send_Data_To_Screen1(ADDR_KILL_REASON,E_TEMP_ERROR);
		break;
	case B_TEMP_KILL:
		LGT_Send_Data_To_Screen1(ADDR_KILL_REASON, B_TEMP_ERROR);
		break;
	case M112_KILL:
		LGT_Send_Data_To_Screen1(ADDR_KILL_REASON, M112_ERROR);
		break;
	case SDCARD_KILL:
		LGT_Send_Data_To_Screen1(ADDR_KILL_REASON,SDCARD_ERROR);
		break;
	case HOME_KILL:
		LGT_Send_Data_To_Screen1(ADDR_KILL_REASON,HOME_FAILE);
		break;
	case TIMEOUT_KILL:
		LGT_Send_Data_To_Screen1(ADDR_KILL_REASON,TIMEOUT_ERROR);
		break;
	case EXTRUDER_KILL:
		LGT_Send_Data_To_Screen1(ADDR_KILL_REASON,EXTRUDER_NUM_ERROR);
		break;
	case DRIVER_KILL:
		LGT_Send_Data_To_Screen1(ADDR_KILL_REASON,DRIVER_ERROR);
		break;
	case E_MINTEMP_KILL:
		LGT_Send_Data_To_Screen1(ADDR_KILL_REASON, E_MINTEMP_ERROR);
		break;
	case B_MINTEMP_KILL:
		LGT_Send_Data_To_Screen1(ADDR_KILL_REASON, B_MINTEMP_ERROR);
		break;
	case E_MAXTEMP_KILL:
		LGT_Send_Data_To_Screen1(ADDR_KILL_REASON, E_MAXTEMP_ERROR);
		break;
	case B_MAXTEMP_KILL:
		LGT_Send_Data_To_Screen1(ADDR_KILL_REASON, B_MAXTEMP_ERROR);
		break;
	case E_RUNAWAY_KILL:
		LGT_Send_Data_To_Screen1(ADDR_KILL_REASON, E_RUNAWAY_ERROR);
		break;
	default:
		break;
	}
}
/*************************************
FUNCTION:	Getting and saving commands of MYSERIAL1(DWIN_Screen)
**************************************/
void LGT_SCR::LGT_Get_MYSERIAL1_Cmd()
{
	memset(data_storage, 0, sizeof(data_storage));
	while (re_count<DATA_SIZE &&MYSERIAL1.available() > 0)
	{
		data_storage[re_count] = MYSERIAL1.read();
		if (data_storage[0] != DW_FH_0)
		{
			continue;
		}
		delay(5);
		re_count++;
	}
	if (re_count < 1)    //null
		return;
	else if (re_count >= 2 && (Rec_Data.head[0] == data_storage[0]) && (Rec_Data.head[1] == data_storage[1]))
	{
		Rec_Data.cmd = data_storage[3];
		if (Rec_Data.cmd == DW_CMD_VAR_R)
		{
			Rec_Data.addr = data_storage[4];
			Rec_Data.addr = (Rec_Data.addr << 8) | data_storage[5];
			Rec_Data.datalen = data_storage[6];
			for (int i = 0; i < Rec_Data.datalen; i = i + 2)
			{
				Rec_Data.data[i / 2] = data_storage[7 + i];
				Rec_Data.data[i / 2] = (Rec_Data.data[i / 2] << 8) | data_storage[8 + i];
			}
			LGT_Analysis_DWIN_Screen_Cmd();
			re_count = 0;
			memset(data_storage, 0, sizeof(data_storage));
			memset(&Rec_Data,0,sizeof(Rec_Data));
			Rec_Data.head[0] = DW_FH_0;
			Rec_Data.head[1] = DW_FH_1;
		}
		else
		{
			re_count = 0;
			memset(data_storage, 0, sizeof(data_storage));
		}
	}
	else
	{
		re_count = 0;
		memset(data_storage, 0, sizeof(data_storage));
	}
}
void LGT_SCR::LGT_Stop_Printing()
{
		card.stopSDPrint(
#if SD_RESORT
			true
#endif
		);
		clear_command_queue();
		quickstop_stepper();
		delay(100);
		print_job_timer.stop();
		thermalManager.disable_all_heaters();
	#if FAN_COUNT > 0
			for (uint8_t i = 0; i < FAN_COUNT; i++) fanSpeeds[i] = 0;
	#endif
			wait_for_heatup = false;
	#if ENABLED(POWER_LOSS_RECOVERY)
			card.openJobRecoveryFile();
			job_recovery_info.valid_head =0;
			job_recovery_info.valid_foot =0;
			(void)card.saveJobRecoveryInfo();
			card.closeJobRecoveryFile();
			job_recovery_commands_count = 0;
	#endif
			enqueue_and_echo_commands_P(PSTR("G91"));
			enqueue_and_echo_commands_P(PSTR("G1 Z10"));
			enqueue_and_echo_commands_P(PSTR("G28 X0"));
			enqueue_and_echo_commands_P(PSTR("M2000"));
}
void LGT_SCR::LGT_Exit_Print_Page()   //return to home menu
{
	feedrate_percentage = 100;
	planner.flow_percentage[0] = 100;
	LGT_Clean_DW_Display_Data(ADDR_TXT_HOME_FILE_NAME);
	LGT_Clean_DW_Display_Data(ADDR_VAL_FAN);
	LGT_Send_Data_To_Screen(ADDR_VAL_HOME_PROGRESS, 0);
	recovery_time = 0;
	recovery_percent = 0;
	recovery_z_height = 0.0;
	recovery_E_len = 0.0;
	menu_type = eMENU_HOME;
	LGT_Printer_Data_Updata();
	status_type = PRINTER_STANDBY;
	idle();
	planner.set_e_position_mm((destination[E_CART] = current_position[E_CART] = 0));
}
int LGT_SCR::LGT_Get_Extrude_Temp()
{
	if (fila_type == 0)
	{
		return (PLA_E_TEMP - 5);
	}
	else
		return (ABS_E_TEMP - 5);
}
/*************************************
FUNCTION:	Analysising the commands of DWIN_Screen
**************************************/
void LGT_SCR::LGT_Analysis_DWIN_Screen_Cmd()
{
	uint16_t LGT_feedrate = 0;
	switch (Rec_Data.addr)
	{
	case ADDR_VAL_PRINT_FILE_SELECT:   //Selecting gocede file and displaying on screen
		if (Rec_Data.data[0] < gcode_num && Rec_Data.data[0] != sel_fileid)	// scope verificaion
		{
			DEHILIGHT_FILE_NAME();
			sel_fileid = Rec_Data.data[0];
			LGT_MAC_Send_Filename(ADDR_TXT_PRINT_FILE_SELECT, gcode_id[sel_fileid]);
			HILIGHT_FILE_NAME();
		}
		break;
	case ADDR_VAL_TAR_E:
			thermalManager.setTargetHotend(Rec_Data.data[0], target_extruder);
		break;
	case ADDR_VAL_TAR_B:
		  thermalManager.setTargetBed(Rec_Data.data[0]);
		break;
	case ADDR_VAL_UTILI_FILA_CHANGE_LEN:
		filament_len = Rec_Data.data[0];
		break;
	case ADDR_VAL_FILA_CHANGE_TEMP:
		filament_temp = Rec_Data.data[0];
		break;	
	case ADDR_TXT_ABOUT_MAC_TIME:
		total_print_time = eeprom_read_dword((const uint32_t*)EEPROM_INDEX);
		LGT_Total_Time_To_String(printer_work_time, total_print_time);
		LGT_Send_Data_To_Screen1(ADDR_TXT_ABOUT_WORK_TIME_MAC, printer_work_time);
		break;
	case ADDR_VAL_BUTTON_KEY:
		switch ((E_BUTTON_KEY)Rec_Data.data[0])
		{
		case eBT_MOVE_X_PLUS_0:
			if (current_position[X_AXIS] < X_MAX_POS) {
				current_position[X_AXIS] = current_position[X_AXIS] + 10;
				if (current_position[X_AXIS] > X_MAX_POS)
					current_position[X_AXIS] = X_MAX_POS;
				LGT_Line_To_Current(X_AXIS);
			}
			break;
		case eBT_MOVE_X_MINUS_0:
//			if (current_position[X_AXIS] > X_MIN_POS) {
				current_position[X_AXIS] = current_position[X_AXIS] - 10;
#ifdef U20_Pro
				if (xy_home == true)
#else  //U30_Pro
				if (xyz_home == true || xy_home == true)
#endif
				{
					if (current_position[X_AXIS] < X_MIN_POS)
						current_position[X_AXIS] = X_MIN_POS;
				}
				LGT_Line_To_Current(X_AXIS);
//			}
			break;
		case eBT_MOVE_X_PLUS_1:
			if (current_position[X_AXIS] < X_MAX_POS) {
				current_position[X_AXIS] = current_position[X_AXIS] + 1;
				if (current_position[X_AXIS] > X_MAX_POS)
					current_position[X_AXIS] = X_MAX_POS;
				LGT_Line_To_Current(X_AXIS);
			}
			break;
		case eBT_MOVE_X_MINUS_1:
//			if (current_position[X_AXIS] > X_MIN_POS) {
				current_position[X_AXIS] = current_position[X_AXIS] - 1;
#ifdef U20_Pro
				if (xy_home == true)
#else  //U30_Pro
				if (xyz_home == true || xy_home == true)
#endif
				{
					if (current_position[X_AXIS] < X_MIN_POS)
						current_position[X_AXIS] = X_MIN_POS;
				}
				LGT_Line_To_Current(X_AXIS);
//			}
			break;
		case eBT_MOVE_X_PLUS_2:
			if (current_position[X_AXIS] < X_MAX_POS) {
				current_position[X_AXIS] = current_position[X_AXIS] + 0.1;
				if (current_position[X_AXIS] > X_MAX_POS)
					current_position[X_AXIS] = X_MAX_POS;
				LGT_Line_To_Current(X_AXIS);
			}
			break;
		case eBT_MOVE_X_MINUS_2:
//			if (current_position[X_AXIS] > X_MIN_POS) {

				current_position[X_AXIS] = current_position[X_AXIS] - 0.1;
#ifdef U20_Pro
				if (xy_home == true)
#else  //U30_Pro
				if (xyz_home == true || xy_home == true)
#endif
				{
					if (current_position[X_AXIS] < X_MIN_POS)
						current_position[X_AXIS] = X_MIN_POS;
				}
				LGT_Line_To_Current(X_AXIS);
//			}
			break;
			//Y Axis
		case eBT_MOVE_Y_PLUS_0:
			if (current_position[Y_AXIS] < Y_MAX_POS) {
				current_position[Y_AXIS] = current_position[Y_AXIS] + 10;
				if (current_position[Y_AXIS] > Y_MAX_POS)
					current_position[Y_AXIS] = Y_MAX_POS;
				LGT_Line_To_Current(Y_AXIS);
			}
			break;
		case eBT_MOVE_Y_MINUS_0:
//			if (current_position[Y_AXIS] > Y_MIN_POS) {

				current_position[Y_AXIS] = current_position[Y_AXIS] - 10;
#ifdef U20_Pro
				if (xy_home == true)
#else  //U30_Pro
				if (xyz_home == true || xy_home == true)
#endif
				{
					if (current_position[Y_AXIS] < Y_MIN_POS)
						current_position[Y_AXIS] = Y_MIN_POS;
				}
				LGT_Line_To_Current(Y_AXIS);
//			}
			break;
		case eBT_MOVE_Y_PLUS_1:
			if (current_position[Y_AXIS] < Y_MAX_POS) {
				current_position[Y_AXIS] = current_position[Y_AXIS] + 1;
				if (current_position[Y_AXIS] > Y_MAX_POS)
					current_position[Y_AXIS] = Y_MAX_POS;
				LGT_Line_To_Current(Y_AXIS);
			}
			break;
		case eBT_MOVE_Y_MINUS_1:
//			if (current_position[Y_AXIS] > Y_MIN_POS) {
				current_position[Y_AXIS] = current_position[Y_AXIS] - 1;
#ifdef U20_Pro
				if (xy_home == true)
#else  //U30_Pro
				if (xyz_home == true || xy_home == true)
#endif
				{
					if (current_position[Y_AXIS] < Y_MIN_POS)
						current_position[Y_AXIS] = Y_MIN_POS;
				}
				LGT_Line_To_Current(Y_AXIS);
//			}
			break;
		case eBT_MOVE_Y_PLUS_2:
			if (current_position[Y_AXIS] < Y_MAX_POS) {
				current_position[Y_AXIS] = current_position[Y_AXIS] + 0.1;
				if (current_position[Y_AXIS] > Y_MAX_POS)
					current_position[Y_AXIS] = Y_MAX_POS;
				LGT_Line_To_Current(Y_AXIS);
			}
			break;
		case eBT_MOVE_Y_MINUS_2:
//			if (current_position[Y_AXIS] > Y_MIN_POS) {
				current_position[Y_AXIS] = current_position[Y_AXIS] - 0.1;
#ifdef U20_Pro
				if (xy_home == true)
#else  //U30_Pro
				if (xyz_home == true || xy_home == true)
#endif
				{
					if (current_position[Y_AXIS] < Y_MIN_POS)
						current_position[Y_AXIS] = Y_MIN_POS;
				}
				LGT_Line_To_Current(Y_AXIS);
//			}
			break;
			//Z Axis
		case eBT_MOVE_Z_PLUS_0:
			if (current_position[Z_AXIS] < Z_MAX_POS) {
				current_position[Z_AXIS] = current_position[Z_AXIS] + 10;
				if (current_position[Z_AXIS] > Z_MAX_POS)
					current_position[Z_AXIS] = Z_MAX_POS;
				LGT_Line_To_Current(Z_AXIS);

#ifdef U20_Pro
				if (menu_type != eMENU_MOVE)
				{
					level_z_height = 10 + level_z_height;
					LGT_Send_Data_To_Screen(ADDR_VAL_LEVEL_Z_UP_DOWN, (uint16_t)(10 * level_z_height));
				}
#endif // U20_Pro
			}
			break;
		case eBT_MOVE_Z_MINUS_0:
//			if (current_position[Z_AXIS] > Z_MIN_POS) {
				current_position[Z_AXIS] = current_position[Z_AXIS] - 10;
#ifdef U30_Pro
				if (xyz_home == true || z_home == true)
				{
					if (current_position[Z_AXIS] < Z_MIN_POS)
						current_position[Z_AXIS] = Z_MIN_POS;
				}
#endif // U30_Pro
				LGT_Line_To_Current(Z_AXIS);

#ifdef U20_Pro
				if (menu_type != eMENU_MOVE)
				{
					level_z_height = level_z_height - 10;
					LGT_Send_Data_To_Screen(ADDR_VAL_LEVEL_Z_UP_DOWN, (uint16_t)(10 * level_z_height));
				}
#endif // U20_Pro
//			}
			break;
		case eBT_MOVE_Z_PLUS_1:
			if (current_position[Z_AXIS] < Z_MAX_POS) {
				current_position[Z_AXIS] = current_position[Z_AXIS] + 1;
				if (current_position[Z_AXIS] > Z_MAX_POS)
					current_position[Z_AXIS] = Z_MAX_POS;
				LGT_Line_To_Current(Z_AXIS);

#ifdef U20_Pro
				if (menu_type != eMENU_MOVE)
				{
					level_z_height = level_z_height + 1;
					LGT_Send_Data_To_Screen(ADDR_VAL_LEVEL_Z_UP_DOWN, (uint16_t)(10 * level_z_height));
				}
#endif // U20_Pro
			}
			break;
		case eBT_MOVE_Z_MINUS_1:
//			if (current_position[Z_AXIS] > Z_MIN_POS) {

				current_position[Z_AXIS] = current_position[Z_AXIS] - 1;
#ifdef U30_Pro
				if (xyz_home == true || z_home == true)
				{
					if (current_position[Z_AXIS] < Z_MIN_POS)
						current_position[Z_AXIS] = Z_MIN_POS;
				}
#endif // U30_Pro
				LGT_Line_To_Current(Z_AXIS);

#ifdef U20_Pro
				if (menu_type != eMENU_MOVE)
				{
					level_z_height = level_z_height - 1;
					LGT_Send_Data_To_Screen(ADDR_VAL_LEVEL_Z_UP_DOWN, (uint16_t)(10 * level_z_height));
			}
#endif // U20_Pro
//			}
			break;
		case eBT_MOVE_Z_PLUS_2:
			if (current_position[Z_AXIS] < Z_MAX_POS) {
				current_position[Z_AXIS] = current_position[Z_AXIS] + 0.1;
				if (current_position[Z_AXIS] > Z_MAX_POS)
					current_position[Z_AXIS] = Z_MAX_POS;
				LGT_Line_To_Current(Z_AXIS);

#ifdef U20_Pro
				if (menu_type != eMENU_MOVE)
				{
					level_z_height = level_z_height + 0.1;
					LGT_Send_Data_To_Screen(ADDR_VAL_LEVEL_Z_UP_DOWN, (uint16_t)(10 * level_z_height));
				}
#endif // U20_Pro
			}
			break;
		case eBT_MOVE_Z_MINUS_2:
//			if (current_position[Z_AXIS] > Z_MIN_POS) {
				current_position[Z_AXIS] = current_position[Z_AXIS] - 0.1;
#ifdef U30_Pro
				if (xyz_home == true || z_home == true)
				{
					if (current_position[Z_AXIS] < Z_MIN_POS)
						current_position[Z_AXIS] = Z_MIN_POS;
				}
#endif // U30_Pro
				LGT_Line_To_Current(Z_AXIS);

#ifdef U20_Pro
				if (menu_type != eMENU_MOVE)
				{
					level_z_height = level_z_height - 0.1;
					LGT_Send_Data_To_Screen(ADDR_VAL_LEVEL_Z_UP_DOWN, (uint16_t)(10 * level_z_height));
				}
#endif // U20_Pro
//			}
			break;
			//E Axis
		case eBT_MOVE_E_PLUS_0:
			if (thermalManager.current_temperature[0] >= LGT_Get_Extrude_Temp())
			{
				current_position[E_AXIS] = current_position[E_AXIS]+10;
				LGT_Line_To_Current(E_AXIS);
			}
			else
			{
				menu_move_dis_chk = 0;
				LGT_Send_Data_To_Screen(ADDR_VAL_EXTRUDE_TEMP, LGT_Get_Extrude_Temp());
				LGT_Change_Page(ID_DIALOG_MOVE_NO_TEMP);
			}
			break;
		case eBT_MOVE_E_MINUS_0:
			if (thermalManager.current_temperature[0] >= LGT_Get_Extrude_Temp())
			{
				current_position[E_AXIS] = current_position[E_AXIS]-10;
				LGT_Line_To_Current(E_AXIS);
			}
			else
			{
				menu_move_dis_chk = 0;
				LGT_Send_Data_To_Screen(ADDR_VAL_EXTRUDE_TEMP, LGT_Get_Extrude_Temp());
				LGT_Change_Page(ID_DIALOG_MOVE_NO_TEMP);
			}
			break;
		case eBT_MOVE_E_PLUS_1:
			if (thermalManager.current_temperature[0] >= LGT_Get_Extrude_Temp())
			{
				current_position[E_AXIS] = current_position[E_AXIS]+1;
				LGT_Line_To_Current(E_AXIS);
			}
			else
			{
				menu_move_dis_chk = 1;
				LGT_Send_Data_To_Screen(ADDR_VAL_EXTRUDE_TEMP, LGT_Get_Extrude_Temp());
				LGT_Change_Page(ID_DIALOG_MOVE_NO_TEMP);
			}
			break;
		case eBT_MOVE_E_MINUS_1:
			if (thermalManager.current_temperature[0] >= LGT_Get_Extrude_Temp())
			{
				current_position[E_AXIS] = current_position[E_AXIS]-1;
				LGT_Line_To_Current(E_AXIS);
			}
			else
			{
				menu_move_dis_chk = 1;
				LGT_Send_Data_To_Screen(ADDR_VAL_EXTRUDE_TEMP, LGT_Get_Extrude_Temp());
				LGT_Change_Page(ID_DIALOG_MOVE_NO_TEMP);
			}
			break;
		case eBT_MOVE_E_PLUS_2:
			if (thermalManager.current_temperature[0] >= LGT_Get_Extrude_Temp())
			{
				current_position[E_AXIS] = current_position[E_AXIS]+0.1;
				LGT_Line_To_Current(E_AXIS);
			}
			else
			{
				menu_move_dis_chk = 2;
				LGT_Send_Data_To_Screen(ADDR_VAL_EXTRUDE_TEMP, LGT_Get_Extrude_Temp());
				LGT_Change_Page(ID_DIALOG_MOVE_NO_TEMP);
			}
			break;
		case eBT_MOVE_E_MINUS_2:
			if (thermalManager.current_temperature[0] >= LGT_Get_Extrude_Temp())
			{
				current_position[E_AXIS] = current_position[E_AXIS]-0.1;
				LGT_Line_To_Current(E_AXIS);
			}
			else
			{
				menu_move_dis_chk = 2;
				LGT_Send_Data_To_Screen(ADDR_VAL_EXTRUDE_TEMP, LGT_Get_Extrude_Temp());
				LGT_Change_Page(ID_DIALOG_MOVE_NO_TEMP);
			}
			break;
			//Axis Homing
		case eBT_MOVE_XY_HOME:
			delay(5);
			enqueue_and_echo_commands_P(PSTR("G28 X0 Y0"));
			xy_home = true;
			break;
		case eBT_MOVE_Z_HOME:
			delay(5);
			#ifdef U20_Pro
				enqueue_and_echo_commands_P(PSTR("G28"));
				xy_home = true;
			#else
				enqueue_and_echo_commands_P(PSTR("G28 Z0")); //U30_Pro
				z_home = true;
			#endif
			break;
		case eBT_DIAL_MOVE_NO_TEMP_RET:   // ok button 
			if (menu_move_dis_chk == 0)
				LGT_Change_Page(ID_MENU_MOVE_0);
			else // equal to 1 or 2
				LGT_Change_Page(ID_MENU_MOVE_1 - 1 + menu_move_dis_chk);
			break;
		case eBT_DIAL_FILA_NO_TEMP_RET:
			if (menu_type == eMENU_UTILI_FILA) {
				LGT_Change_Page(ID_MENU_UTILI_FILA_0 + menu_fila_type_chk);
			}
			else {   // menu_type == eMENU_HOME_FILA
				LGT_Change_Page(ID_MENU_HOME_FILA_0 + menu_fila_type_chk);
			}
			break;
		case eBT_PRINT_FILE_OPEN:
			if (sel_fileid >-1)
			{
				uint8_t i = sel_fileid / 5;
				if (i == 0)
					LGT_Change_Page(ID_DIALOG_PRINT_START_0);
				else
				{
					LGT_Change_Page(ID_DIALOG_PRINT_START_1 - 1 + i);
				}
			}
			break;
		case eBT_PRINT_FILE_OPEN_YES:
			if (sel_fileid > -1)
			{
					card.getfilename(gcode_id[sel_fileid]);
					card.openFile(card.filename,true);
					card.startFileprint();
					print_job_timer.start();		
					LGT_MAC_Send_Filename(ADDR_TXT_HOME_FILE_NAME, gcode_id[sel_fileid]);
					delay(5);
					menu_type = eMENU_PRINT_HOME;
					LGT_Printer_Data_Updata();
					status_type = PRINTER_PRINTING;
					LGT_is_printing = true;
					LGT_Send_Data_To_Screen(ADDR_VAL_ICON_HIDE, 0);
					idle();
					LGT_Change_Page(ID_MENU_PRINT_HOME);
					fila_type = 0;    //PLA
					LGT_Save_Recovery_Filename(DW_CMD_VAR_W, DW_FH_1, ADDR_TXT_HOME_FILE_NAME,32);
			}
			break;
		case eBT_MOVE_DISABLE:
			clear_command_queue();
			quickstop_stepper();
//			enqueue_and_echo_commands_P(PSTR("M84"));
			break;
		case eBT_MOVE_ENABLE:
				enable_all_steppers();
			break;

		case eBT_PRINT_HOME_PAUSE:
			LGT_Change_Page(ID_DIALOG_PRINT_WAIT);
			status_type = PRINTER_PAUSE;
			card.pauseSDPrint();
			print_job_timer.pause();
			enqueue_and_echo_commands_P(PSTR("M2001"));
			break;
		case eBT_PRINT_HOME_RESUME:
				LGT_Change_Page(ID_MENU_PRINT_HOME);
			    do_blocking_move_to_xy(resume_x_position,resume_y_position,50); 
			    card.startFileprint();
				print_job_timer.start();
		        runout.reset();
				menu_type = eMENU_PRINT_HOME;
				status_type = PRINTER_PRINTING;
			break;
		case eBT_PRINT_HOME_ABORT:
				LGT_Change_Page(ID_DIALOG_PRINT_WAIT);
				wait_for_heatup = false;
				LGT_stop_printing = true;
				LGT_Printer_Total_Work_Time();
				LGT_Exit_Print_Page();
			break;
		case eBT_PRINT_HOME_FINISH:
				runout.reset();
				LGT_Change_Page(ID_MENU_HOME);
				LGT_Exit_Print_Page();
				LGT_is_printing = false;
			break;
		case eBT_UTILI_FILA_PLA:
			menu_fila_type_chk = 1;
			fila_type = 0;
			thermalManager.setTargetHotend(PLA_E_TEMP, target_extruder);
			status_type = PRINTER_HEAT;
			thermalManager.setTargetBed(PLA_B_TEMP);
			LGT_Send_Data_To_Screen(ADDR_VAL_TAR_E, thermalManager.target_temperature[0]);
			delayMicroseconds(1);
			LGT_Send_Data_To_Screen(ADDR_VAL_TAR_B, thermalManager.target_temperature_bed);
			LGT_Send_Data_To_Screen(ADDR_VAL_FILA_CHANGE_TEMP, thermalManager.target_temperature[0]);
			break;
		case eBT_UTILI_FILA_ABS:
			fila_type = 1;
			menu_fila_type_chk = 2;
			thermalManager.setTargetHotend(ABS_E_TEMP, target_extruder);
			status_type = PRINTER_HEAT;
			thermalManager.setTargetBed(ABS_B_TEMP);
			LGT_Send_Data_To_Screen(ADDR_VAL_TAR_E, thermalManager.target_temperature[0]);
			delayMicroseconds(1);
			LGT_Send_Data_To_Screen(ADDR_VAL_TAR_B, thermalManager.target_temperature_bed);
			LGT_Send_Data_To_Screen(ADDR_VAL_FILA_CHANGE_TEMP, thermalManager.target_temperature[0]);
			break;
		case eBT_UTILI_FILA_LOAD:
				if (thermalManager.current_temperature[0] >= (filament_temp - 5))
				{
					enqueue_and_echo_commands_P(PSTR("M2004"));
				}
				else
				{
					memset(cmd_E, 0, sizeof(cmd_E));
					if (menu_type == eMENU_UTILI_FILA)
					{
						LGT_Change_Page(ID_DIALOG_UTILI_FILA_WAIT);
					}
					else if (menu_type == eMENU_HOME_FILA)
					{
						LGT_Change_Page(ID_DIALOG_PRINT_FILA_WAIT);
					}
					sprintf_P(cmd_E, PSTR("M109 S%i"), filament_temp);
					enqueue_and_echo_command(cmd_E);
					enqueue_and_echo_commands_P(PSTR("M2004"));
				}
			break;
		case eBT_UTILI_FILA_UNLOAD:
			if (thermalManager.current_temperature[0] >= (filament_temp - 5))
			{
				enqueue_and_echo_commands_P(PSTR("M2005"));
			}
			else
			{
				memset(cmd_E, 0, sizeof(cmd_E));
				LGT_Change_Page(ID_DIALOG_UTILI_FILA_WAIT);
				sprintf_P(cmd_E, PSTR("M109 S%i"), filament_temp);
				enqueue_and_echo_command(cmd_E);
				enqueue_and_echo_commands_P(PSTR("M2005"));
			}
			break;
		case eBT_PRINT_FILA_HEAT_NO:
			clear_command_queue();
			wait_for_heatup = false;
			if (menu_type == eMENU_UTILI_FILA)
			{
				thermalManager.disable_all_heaters();
				LGT_Change_Page(ID_MENU_UTILI_FILA_0 + menu_fila_type_chk);
			}
			else if (menu_type == eMENU_HOME_FILA)
			{
				planner.set_e_position_mm((destination[E_CART] = current_position[E_CART] = (resume_e_position-2)));
				LGT_Change_Page(ID_MENU_HOME_FILA_0);
			}
			break;
		case eBT_PRINT_FILA_UNLOAD_OK:
			clear_command_queue();
			quickstop_stepper();
			delay(5);
			if (menu_type == eMENU_UTILI_FILA)
			{
				LGT_Change_Page(ID_MENU_UTILI_FILA_0 + menu_fila_type_chk);
			}
			else if (menu_type == eMENU_HOME_FILA)
			{
				LGT_Change_Page(ID_MENU_HOME_FILA_0);
			}
			break;
		case eBT_PRINT_FILA_LOAD_OK:
			clear_command_queue();
			quickstop_stepper();
			delay(5);
			if (menu_type == eMENU_UTILI_FILA)
			{
				LGT_Change_Page(ID_MENU_UTILI_FILA_0 + menu_fila_type_chk);
			}
			else if (menu_type == eMENU_HOME_FILA)
			{
				LGT_Change_Page(ID_DIALOG_LOAD_FINISH);
			}
			break;
		case eBT_PRINT_FILA_CHANGE_YES:
			if(menu_type==eMENU_PRINT_HOME)
				LGT_Change_Page(ID_DIALOG_PRINT_WAIT);
			else if(menu_type== eMENU_TUNE)
				LGT_Change_Page(ID_DIALOG_PRINT_TUNE_WAIT);
			status_type = PRINTER_PAUSE;
			card.pauseSDPrint();
			print_job_timer.pause();
			enqueue_and_echo_commands_P(PSTR("M2006"));
			break;
		case eBT_HOME_RECOVERY_YES:
			LGT_Send_Data_To_Screen(ADDR_VAL_ICON_HIDE, 0);
			return_home = false;
			#ifdef U20_Pro
				status_type = PRINTER_PRINTING;
			#endif // U20_Pro
			delay(5);
		#if ENABLED(POWER_LOSS_RECOVERY)
			LGT_is_printing = true;
			LGT_Save_Recovery_Filename(DW_CMD_VAR_W, DW_FH_0, ADDR_TXT_HOME_FILE_NAME, 32);
			LGT_Power_Loss_Recovery_Resume();
			menu_type = eMENU_PRINT_HOME;
			LGT_Printer_Data_Updata();
			LGT_Change_Page(ID_MENU_PRINT_HOME);
		#endif
			break;
		case eBT_HOME_RECOVERY_NO:
			total_print_time = total_print_time+job_recovery_info.print_job_elapsed/60;
			eeprom_write_dword((uint32_t*)EEPROM_INDEX, total_print_time);

			#if ENABLED(POWER_LOSS_RECOVERY)
				card.removeJobRecoveryFile();
			#endif
				disable_Z();
				return_home = false;
				recovery_time = 0;
				recovery_percent = 0;
				recovery_z_height = 0.0;
				recovery_E_len = 0.0;
				LGT_Change_Page(ID_MENU_HOME);
				menu_type = eMENU_HOME;
			break;
		case eBT_PRINT_FILE_CLEAN: //Cleaning sel_fileid
			if (sel_fileid > -1)
			{
				DEHILIGHT_FILE_NAME();
				sel_fileid = -1;
				LGT_Clean_DW_Display_Data(ADDR_TXT_PRINT_FILE_SELECT); //Cleaning sel_file txt
				LGT_Clean_DW_Display_Data(ADDR_TXT_HOME_ELAP_TIME);    //Cleaning time
			}
			menu_type = eMENU_FILE;
			break;
//////////////////////////////////////////////////////////////////////////
		case eBT_UTILI_LEVEL_CORNER_POS_1:
			#ifdef U20_Pro
				if (xy_home == false)
				{
					thermalManager.setTargetHotend(0, target_extruder);
					thermalManager.setTargetBed(0);
					enqueue_and_echo_commands_P(PSTR("G28 X0 Y0"));
					xy_home = true;
				}
				enqueue_and_echo_commands_P(PSTR("G1 X50 Y50"));
			#else  //U30_Pro
				if (xyz_home == false)
				{
					thermalManager.setTargetHotend(0, target_extruder);
					thermalManager.setTargetBed(0);
					enqueue_and_echo_commands_P(PSTR("G28"));
					xyz_home = true;
				}
				enqueue_and_echo_commands_P(PSTR("G1 Z10"));
				enqueue_and_echo_commands_P(PSTR("G1 X30 Y30 F3000"));
				enqueue_and_echo_commands_P(PSTR("G1 Z0"));
			#endif
			break;
		case eBT_UTILI_LEVEL_CORNER_POS_2: //45 002D
			#ifdef U20_Pro
				if (xy_home == false)
				{
					thermalManager.setTargetHotend(0, target_extruder);
					thermalManager.setTargetBed(0);
					enqueue_and_echo_commands_P(PSTR("G28 X0 Y0"));
					xy_home = true;
				}
				enqueue_and_echo_commands_P(PSTR("G1 X250 Y50"));
			#else  //U30_Pro
				if (xyz_home == false)
				{
					thermalManager.setTargetHotend(0, target_extruder);
					thermalManager.setTargetBed(0);
					enqueue_and_echo_commands_P(PSTR("G28"));
					xyz_home = true;
				}
				enqueue_and_echo_commands_P(PSTR("G1 Z10"));
				enqueue_and_echo_commands_P(PSTR("G1 X190 Y30 F3000"));
				enqueue_and_echo_commands_P(PSTR("G1 Z0"));
			#endif
			break;
		case eBT_UTILI_LEVEL_CORNER_POS_3:
			#ifdef U20_Pro
				if (xy_home == false)
				{
					thermalManager.setTargetHotend(0, target_extruder);
					thermalManager.setTargetBed(0);
					enqueue_and_echo_commands_P(PSTR("G28 X0 Y0"));
					xy_home = true;
				}
				enqueue_and_echo_commands_P(PSTR("G1 X250 Y250"));
			#else  //U30_Pro
				if (xyz_home == false)
				{
					thermalManager.setTargetHotend(0, target_extruder);
					thermalManager.setTargetBed(0);
					enqueue_and_echo_commands_P(PSTR("G28"));
					xyz_home = true;
				}
				enqueue_and_echo_commands_P(PSTR("G1 Z10"));
				enqueue_and_echo_commands_P(PSTR("G1 X190 Y190 F3000"));
				enqueue_and_echo_commands_P(PSTR("G1 Z0"));
			#endif
			break;
		case eBT_UTILI_LEVEL_CORNER_POS_4:
			#ifdef U20_Pro
				if (xy_home == false)
				{
					thermalManager.setTargetHotend(0, target_extruder);
					thermalManager.setTargetBed(0);
					enqueue_and_echo_commands_P(PSTR("G28 X0 Y0"));
					xy_home = true;
				}
				enqueue_and_echo_commands_P(PSTR("G1 X50 Y250"));
			#else  //U30_Pro
				if (xyz_home == false)
				{
					thermalManager.setTargetHotend(0, target_extruder);
					thermalManager.setTargetBed(0);
					enqueue_and_echo_commands_P(PSTR("G28"));
					xyz_home = true;
				}
				enqueue_and_echo_commands_P(PSTR("G1 Z10"));
				enqueue_and_echo_commands_P(PSTR("G1 X30 Y190 F3000"));
				enqueue_and_echo_commands_P(PSTR("G1 Z0"));
			#endif
			break;
		case eBT_UTILI_LEVEL_CORNER_POS_5:
			#ifdef U20_Pro
				if (xy_home == false)
				{
					thermalManager.setTargetHotend(0, target_extruder);
					thermalManager.setTargetBed(0);
					enqueue_and_echo_commands_P(PSTR("G28 X0 Y0"));
					xy_home = true;
				}
				enqueue_and_echo_commands_P(PSTR("G1 X150 Y150"));
			#else  //U30_Pro
				if (xyz_home == false)
				{
					thermalManager.setTargetHotend(0, target_extruder);
					thermalManager.setTargetBed(0);
					enqueue_and_echo_commands_P(PSTR("G28"));
					xyz_home = true;
				}
				enqueue_and_echo_commands_P(PSTR("G1 Z10"));
				enqueue_and_echo_commands_P(PSTR("G1 X110 Y110 F3000"));
				enqueue_and_echo_commands_P(PSTR("G1 Z0"));
			#endif
			break;
		case eBT_UTILI_LEVEL_CORNER_BACK:
			#ifdef U20_Pro
				if (xy_home) {
					xy_home = false;
					enqueue_and_echo_commands_P(PSTR("G1 Z10"));	//up 10mm to prevent from damaging bed
				}
			#else
				if (xyz_home) {
					xyz_home = false;
					enqueue_and_echo_commands_P(PSTR("G1 Z10"));	//up 10mm to prevent from damaging bed
				}
			#endif
			break;
	#ifdef U20_Pro
		case eBT_UTILI_LEVEL_MEASU_START:  // == PREVIOUS
			LGT_Change_Page(ID_DIALOG_LEVEL_WAIT);
			level_z_height = 0;
			LGT_Send_Data_To_Screen(ADDR_VAL_LEVEL_Z_UP_DOWN,0);
			menu_measu_step = 1;
			menu_measu_dis_chk = 1;
			thermalManager.setTargetHotend(0, target_extruder);
			thermalManager.setTargetBed(0);
			enqueue_and_echo_commands_P(PSTR("G28 X0 Y0"));
//			enqueue_and_echo_commands_P(PSTR("G1 X150 Y150 F3000"));
			enqueue_and_echo_commands_P(PSTR("G1 X180 Y153 F3000"));
			enqueue_and_echo_commands_P(PSTR("M2002"));
			xy_home = true;
			break;
		case eBT_UTILI_LEVEL_MEASU_DIS_0:
			menu_measu_dis_chk = 0;
			break;
		case eBT_UTILI_LEVEL_MEASU_DIS_1:    //50 0032
			menu_measu_dis_chk = 1;
			break;
		case eBT_UTILI_LEVEL_MEASU_S1_NEXT:
			menu_measu_step = 2;
			menu_measu_dis_chk = 1;
			break;
		case eBT_UTILI_LEVEL_MEASU_S2_NEXT:
			menu_measu_step = 3;
			menu_measu_dis_chk = 1;
			settings.reset();
			enqueue_and_echo_commands_P(PSTR("G28"));
			enqueue_and_echo_commands_P(PSTR("G29"));
			break;
		case eBT_UTILI_LEVEL_MEASU_S1_EXIT_NO:
			LGT_Change_Page(ID_MENU_MEASU_S1 + menu_measu_dis_chk);
			break;
		case eBT_UTILI_LEVEL_MEASU_S2_EXIT_NO:
			LGT_Change_Page(ID_MENU_MEASU_S2 + menu_measu_dis_chk);
			break;
		case eBT_UTILI_LEVEL_MEASU_EXIT_OK:
			clear_command_queue();
			quickstop_stepper();
			enqueue_and_echo_commands_P(PSTR("M18"));
			break;
		case eBT_UTILI_LEVEL_MEASU_S3_EXIT_NO:
			LGT_Change_Page(ID_MENU_MEASU_S3);
			break;
		case eBT_UTILI_LEVEL_MEASU_STOP_MOVE:
			level_z_height = 0;
			LGT_Send_Data_To_Screen(ADDR_VAL_LEVEL_Z_UP_DOWN, 0);
			clear_command_queue();
			quickstop_stepper();
			enqueue_and_echo_commands_P(PSTR("M17"));
			break;
	#endif  //U20_Pro
		case eBT_MOVE_P0:
			menu_move_dis_chk = 0;
			break;
		case eBT_MOVE_P1:
			menu_move_dis_chk = 1;
			break;
		case eBT_MOVE_P2:
			menu_move_dis_chk = 2;
			break;

#ifdef U20_Pro
		case eBT_TUNE_SWITCH_LEDS:
			led_on = !led_on;
			if (led_on == false)
			{
				LED_Bright_State(LED_BLUE, 10, 0);  //close LED
				LGT_Send_Data_To_Screen(ADDR_VAL_LEDS_SWITCH, 1);
				delay(5);
			}
			else
			{
				LGT_Send_Data_To_Screen(ADDR_VAL_LEDS_SWITCH, 0);
				delay(5);
			}
			break;
#endif

////////////////////////////////////////////////////////////////////////
		default: break;
		}
		break;
	case ADDR_VAL_FAN:
		fanSpeeds[0] = Rec_Data.data[0];
		break;
	case ADDR_VAL_FEED:
		feedrate_percentage = Rec_Data.data[0];
		LGT_feedrate = (uint16_t)(MMS_SCALED(feedrate_mm_s) * 10);
		if (LGT_feedrate > 3000)
			LGT_feedrate = 3000;
		LGT_Send_Data_To_Screen(ADDR_VAL_CUR_FEED,LGT_feedrate);
		break;
	case ADDR_VAL_FLOW:
		planner.flow_percentage[0] = Rec_Data.data[0];
		break;
	case ADDR_VAL_MENU_TYPE:
		menu_type = (E_MENU_TYPE)(Rec_Data.data[0]);
		LGT_Printer_Data_Updata();
		// write in page handle
		if (menu_type == eMENU_UTILI_FILA || menu_type == eMENU_HOME_FILA)
			menu_fila_type_chk = 0; // clear variable
		else if (menu_type == eMENU_MOVE)
			menu_move_dis_chk = 0; // clear variable
		break;
	default:
		break;
	}
}
void LGT_SCR::LGT_Send_Data_To_Screen(uint16_t Addr, int16_t Num)
{
	    memset(data_storage, 0, sizeof(data_storage));
		data_storage[0] = Send_Data.head[0];
		data_storage[1] = Send_Data.head[1];
		data_storage[2] = 0x05;
		data_storage[3] = DW_CMD_VAR_W;
		data_storage[4] = (unsigned char)(Addr>> 8); //(Num >> 8) & 0xFF
		data_storage[5] = (unsigned char)(Addr & 0x00FF);
		data_storage[6] = (unsigned char)(Num >> 8);
		data_storage[7] = (unsigned char)(Num & 0x00FF);
		for (int i = 0; i < 8; i++)
		{
			MYSERIAL1.print(data_storage[i]);
			delayMicroseconds(1);
		}
}
void LGT_SCR::LGT_Send_Data_To_Screen(unsigned int addr, float num, char axis)
{
	memset(data_storage, 0, sizeof(data_storage));
	memset(Send_Data.data_num, 0, 6);
	dtostrf((double)num, 2, 1, Send_Data.data_num);
	data_storage[0] = Send_Data.head[0];
	data_storage[1] = Send_Data.head[1];
	data_storage[2] = 0x09;
	data_storage[3] = DW_CMD_VAR_W;
	data_storage[4] = (unsigned char)(addr >> 8);
	data_storage[5] = (unsigned char)(addr & 0x00FF);
	for (int i = 0; i < 6; i++)
	{
		data_storage[6 + i] = Send_Data.data_num[i];
	}
	for (int i = 0; i < 12; i++)
	{
		MYSERIAL1.print(data_storage[i]);
		delayMicroseconds(1);
	}
}

void LGT_SCR::LGT_Send_Data_To_Screen(unsigned int addr, char* buf)
{
	memset(data_storage, 0, sizeof(data_storage));
	data_storage[0] = Send_Data.head[0];
	data_storage[1] = Send_Data.head[1];
	data_storage[2] = 0x0A;
	data_storage[3] = DW_CMD_VAR_W;
	data_storage[4] = (unsigned char)(addr >> 8);
	data_storage[5] = (unsigned char)(addr & 0x00FF);
	for (int i = 0; i < 7; i++)
	{
		data_storage[6 + i] = buf[i];
	}
	for (int i = 0; i < 13; i++)

	{
		MYSERIAL1.print(data_storage[i]);
		delayMicroseconds(1);
	}
}
void LGT_SCR::LGT_Send_Data_To_Screen1(unsigned int addr,const char* buf)
{
	memset(data_storage, 0, sizeof(data_storage));
	data_storage[0] = Send_Data.head[0];
	data_storage[1] = Send_Data.head[1];
	data_storage[2] = 0x22;
	data_storage[3] = DW_CMD_VAR_W;
	data_storage[4] = (unsigned char)(addr >> 8);
	data_storage[5] = (unsigned char)(addr & 0x00FF);
	for (int i = 0; i < 31; i++)
	{
		data_storage[6 + i] = buf[i];
	}
	for (int i = 0; i < 37; i++)

	{
		MYSERIAL1.print(data_storage[i]);
		delayMicroseconds(1);
	}
}

void LGT_SCR::LGT_Screen_System_Reset()
{
	memset(data_storage, 0, sizeof(data_storage));
	data_storage[0] = Send_Data.head[0];
	data_storage[1] = Send_Data.head[1];
	data_storage[2] = 0x07;
	data_storage[3] = DW_CMD_VAR_W;
	data_storage[4] = 0x00;
	data_storage[5] = 0x04;
	data_storage[6] = 0x55;
	data_storage[7] = 0xAA;
	data_storage[8] = 0x5A;
	data_storage[9] = 0xA5;
	for (int i = 0; i < 10; i++)
	{
		MYSERIAL1.print(data_storage[i]);
		delayMicroseconds(1);
	}
}
millis_t Next_Temp_Time = 0;
void LGT_SCR::LGT_Main_Function()
{
	LGT_Get_MYSERIAL1_Cmd();
	if (millis() >= Next_Temp_Time)
	{
		Next_Temp_Time += 2000;
		if (tartemp_flag == true)  //M104/M109/M190/140
		{
			tartemp_flag = false;
			if(LGT_is_printing==false)
				status_type = PRINTER_HEAT;
			LGT_Send_Data_To_Screen(ADDR_VAL_TAR_E, thermalManager.target_temperature[0]);
			LGT_Send_Data_To_Screen(ADDR_VAL_TAR_B, thermalManager.target_temperature_bed);
		}
		LGT_Printer_Data_Updata();
		LGT_Get_MYSERIAL1_Cmd();
	}
	#ifdef U20_Pro
		if (led_on == true)
			LGT_Printer_Light_Update();
	#endif // U20_Pro
	LGT_SDCard_Status_Update();
}
void LGT_SCR::LGT_Printer_Data_Updata()
{
	uint8_t progress_percent = 0;
	uint16_t LGT_feedrate = 0;
	switch (menu_type)
	{
	case eMENU_HOME:
		LGT_Send_Data_To_Screen(ADDR_VAL_CUR_E, (int16_t)thermalManager.current_temperature[0]);
		LGT_Send_Data_To_Screen(ADDR_VAL_CUR_B, (int16_t)thermalManager.current_temperature_bed);
		break;
	case eMENU_TUNE:
		LGT_Send_Data_To_Screen(ADDR_VAL_CUR_E, (int16_t)thermalManager.current_temperature[0]);
		LGT_Send_Data_To_Screen(ADDR_VAL_CUR_B, (int16_t)thermalManager.current_temperature_bed);
		LGT_Get_MYSERIAL1_Cmd();
		LGT_Send_Data_To_Screen(ADDR_VAL_FAN,fanSpeeds[0]);
		LGT_Send_Data_To_Screen(ADDR_VAL_FEED,feedrate_percentage);
		LGT_Send_Data_To_Screen(ADDR_VAL_FLOW,planner.flow_percentage[0]);
		break;
	case eMENU_MOVE:
		LGT_Send_Data_To_Screen(ADDR_VAL_MOVE_POS_X, (int16_t)(current_position[X_AXIS] * 10));
		LGT_Send_Data_To_Screen(ADDR_VAL_MOVE_POS_Y, (int16_t)(current_position[Y_AXIS] * 10));
		LGT_Send_Data_To_Screen(ADDR_VAL_MOVE_POS_Z, (int16_t)(current_position[Z_AXIS] * 10));
		LGT_Send_Data_To_Screen(ADDR_VAL_MOVE_POS_E, (int16_t)(current_position[E_AXIS] * 10));
		break;
	case eMENU_TUNE_E:
		LGT_Send_Data_To_Screen(ADDR_VAL_CUR_E, (int16_t)thermalManager.current_temperature[0]);
		break;
	case eMENU_TUNE_B:
		LGT_Send_Data_To_Screen(ADDR_VAL_CUR_B, (int16_t)thermalManager.current_temperature_bed);
		break;
	case eMENU_TUNE_FAN:
		LGT_Send_Data_To_Screen(ADDR_VAL_FAN,fanSpeeds[0]);
		break;
	case eMENU_TUNE_SPEED:
		LGT_Send_Data_To_Screen(ADDR_VAL_FEED,feedrate_percentage);
		LGT_feedrate = (uint16_t)(MMS_SCALED(feedrate_mm_s) * 10);
		if (LGT_feedrate > 3000)
			LGT_feedrate = 3000;
		LGT_Send_Data_To_Screen(ADDR_VAL_CUR_FEED, LGT_feedrate);
		break;
	case eMENU_TUNE_FLOW:
		LGT_Send_Data_To_Screen(ADDR_VAL_FLOW,planner.flow_percentage[0]);
		break;
	case eMENU_UTILI_FILA:
	case eMENU_HOME_FILA:
		LGT_Send_Data_To_Screen(ADDR_VAL_CUR_E, (int16_t)thermalManager.current_temperature[0]);
		LGT_Send_Data_To_Screen(ADDR_VAL_CUR_B, (int16_t)thermalManager.current_temperature_bed);
		break;
	case eMENU_PRINT_HOME:
		progress_percent = card.percentDone();
		if(progress_percent>0)
			LGT_Send_Data_To_Screen(ADDR_VAL_HOME_PROGRESS, (uint16_t)progress_percent);
		else
			LGT_Send_Data_To_Screen(ADDR_VAL_HOME_PROGRESS, (uint16_t)recovery_percent);

		Duration_Time = (print_job_timer.duration()) + recovery_time;
		Duration_Time.toDigital(total_time);
		LGT_Send_Data_To_Screen(ADDR_TXT_HOME_ELAP_TIME,total_time);
		//delay(10);
		LGT_Send_Data_To_Screen(ADDR_VAL_HOME_Z_HEIGHT, (int16_t)((current_position[Z_AXIS] + recovery_z_height) * 10));  //Current Z height
		LGT_Send_Data_To_Screen(ADDR_VAL_CUR_E, (int16_t)thermalManager.current_temperature[0]);
		LGT_Send_Data_To_Screen(ADDR_VAL_CUR_B, (int16_t)thermalManager.current_temperature_bed);
		break;
	default:
		break;
	}
}
void LGT_SCR::LGT_Printer_Light_Update()
{
	if (LGT_is_printing == false&&((thermalManager.current_temperature_bed < thermalManager.target_temperature_bed)
		|| (thermalManager.current_temperature[0] < (TEMP_RANGE+thermalManager.target_temperature[0]))))
	{
		status_type = PRINTER_HEAT;
	}
	else if (LGT_is_printing == false&&((thermalManager.current_temperature_bed >=thermalManager.target_temperature_bed)
		|| (thermalManager.current_temperature[0] > (thermalManager.target_temperature[0]))
		))
	{
		status_type = PRINTER_STANDBY;
	}
		LGT_Printer_Status_Light();
}

void LGT_SCR::LGT_Power_Loss_Recovery_Resume() {
	char cmd[30];
	// Restore all hotend temperatures
	sprintf_P(cmd, PSTR("M190 S%i"), job_recovery_info.target_temperature_bed);
	enqueue_and_echo_command(cmd);
	sprintf_P(cmd, PSTR("M109 S%i"), job_recovery_info.target_temperature[0]);
	enqueue_and_echo_command(cmd);
	// Restore print cooling fan speeds
	for (uint8_t i = 0; i < FAN_COUNT; i++) {
		int16_t f = job_recovery_info.fanSpeeds[i];
		if (f) {
			sprintf_P(cmd, PSTR("M106 P%i S%i"), i, f);
			enqueue_and_echo_command(cmd);
		}
	}
#if HOTENDS > 1
	sprintf_P(cmd, PSTR("T%i"), job_recovery_info.active_hotend);
	enqueue_and_echo_command(cmd);
#endif

	// Start draining the job recovery command queue
	job_recovery_phase = JOB_RECOVERY_YES;
	recovery_time = job_recovery_info.print_job_elapsed;
	recovery_percent = job_recovery_info.have_percentdone;
	recovery_z_height = job_recovery_info.save_current_Z;
}

/*************************************
FUNCTION:	Disable  and enable button of DWIN_Screen
pageid: page n (page 1:0001)
buttonid:button n + return value (button 5+06:0506)
sta:disable or enable  (0000:disable  0001:enable)
**************************************/
void LGT_SCR::LGT_Disable_Enable_Screen_Button(unsigned int pageid, unsigned int buttonid, unsigned int sta) 
{
	memset(data_storage, 0, sizeof(data_storage));
	data_storage[0] = Send_Data.head[0];
	data_storage[1] = Send_Data.head[1];
	data_storage[2] = 0x0B;
	data_storage[3] = DW_CMD_VAR_W;
	data_storage[4] = 0x00;
	data_storage[5] = 0xB0;
	data_storage[6] = 0x5A;
	data_storage[7] = 0xA5;
	data_storage[8] = (unsigned char)(pageid>>8);
	data_storage[9] = (unsigned char)(pageid&0x00FF);
	data_storage[10] = (unsigned char)(buttonid>>8);
	data_storage[11] = (unsigned char)(buttonid & 0x00FF);
	data_storage[12] = (unsigned char)(sta >> 8);
	data_storage[13] = (unsigned char)(sta & 0x00FF);
	for (int i = 0; i < 14; i++)
	{
		MYSERIAL1.print(data_storage[i]);
		delayMicroseconds(1);
	}
}
void LGT_SCR::LGT_Save_Recovery_Filename(unsigned char cmd, unsigned char sys_cmd,unsigned int addr, unsigned int length)
{
	memset(data_storage, 0, sizeof(data_storage));
	data_storage[0] = Send_Data.head[0];
	data_storage[1] = Send_Data.head[1];
	data_storage[2] = 0x0B;
	data_storage[3] = DW_CMD_VAR_W;
	data_storage[4] = 0x00;
	data_storage[5] = 0x08;
	data_storage[6] = sys_cmd;
	data_storage[7] = 0x00;
	data_storage[8] = 0x00;
	data_storage[9] = 0x00;
	data_storage[10] = (unsigned char)(addr >> 8);
	data_storage[11] = (unsigned char)(addr & 0x00FF);
	data_storage[12] = (unsigned char)(length >> 8);
	data_storage[13] = (unsigned char)(length & 0x00FF);
	for (int i = 0; i < 14; i++)
	{
		MYSERIAL1.print(data_storage[i]);
		delayMicroseconds(1);
	}
}
/*************************************
FUNCTION:	The main function of DWIN_Screen
**************************************/
void DWIN_MAIN_FUNCTIONS()
{
	LGT_LCD.LGT_Main_Function();
}

#endif // LGT_MAC