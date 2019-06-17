/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * power_loss_recovery.cpp - Resume an SD print after power-loss
 */

#include "MarlinConfig.h"

#if ENABLED(POWER_LOSS_RECOVERY)

#include "power_loss_recovery.h"

#include "cardreader.h"
#include "planner.h"
#include "printcounter.h"
#include "serial.h"
#include "temperature.h"
#include "ultralcd.h"
#ifdef LGT_MAC
#include "LGT_SCR.h"
extern LGT_SCR LGT_LCD;
extern millis_t recovery_time;
extern float recovery_z_height;
extern float recovery_E_len;
extern bool check_recovery;
#endif // LGT_MAC

// Recovery data
job_recovery_info_t job_recovery_info;
JobRecoveryPhase job_recovery_phase = JOB_RECOVERY_IDLE;
uint8_t job_recovery_commands_count; //=0
char job_recovery_commands[BUFSIZE + APPEND_CMD_COUNT][MAX_CMD_SIZE];
extern uint8_t active_extruder, commands_in_queue, cmd_queue_index_r;

#if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
  void debug_print_job_recovery(const bool recovery) {
    SERIAL_PROTOCOLLNPGM("---- Job Recovery Info ----");
    SERIAL_PROTOCOLPAIR("valid_head:", int(job_recovery_info.valid_head));
    SERIAL_PROTOCOLLNPAIR(" valid_foot:", int(job_recovery_info.valid_foot));
    if (job_recovery_info.valid_head) {
      if (job_recovery_info.valid_head == job_recovery_info.valid_foot) {
        //SERIAL_PROTOCOLPGM("current_position: ");
        //LOOP_XYZE(i) {
        //  SERIAL_PROTOCOL(job_recovery_info.current_position[i]);
        //  if (i < E_AXIS) SERIAL_CHAR(',');
		  SERIAL_PROTOCOL(" ");
		  SERIAL_PROTOCOLPGM("Z_current_position: ");
		  SERIAL_PROTOCOL(job_recovery_info.save_current_Z);
		  SERIAL_PROTOCOL(" ");
		  SERIAL_PROTOCOLPGM("E_current_position: ");
		  SERIAL_PROTOCOL(job_recovery_info.save_current_E);
       // }
        SERIAL_EOL();
		SERIAL_PROTOCOL(" ");
        SERIAL_PROTOCOLLNPAIR("feedrate: ", job_recovery_info.feedrate);

        #if HOTENDS > 1
          SERIAL_PROTOCOLLNPAIR("active_hotend: ", int(job_recovery_info.active_hotend));
        #endif

        SERIAL_PROTOCOLPGM("target_temperature: ");
        HOTEND_LOOP() {
          SERIAL_PROTOCOL(job_recovery_info.target_temperature[e]);
          if (e < HOTENDS - 1) SERIAL_CHAR(',');
        }
        SERIAL_EOL();

        #if HAS_HEATED_BED
          SERIAL_PROTOCOLLNPAIR("target_temperature_bed: ", job_recovery_info.target_temperature_bed);
        #endif

        #if FAN_COUNT
          SERIAL_PROTOCOLPGM("fanSpeeds: ");
          for (int8_t i = 0; i < FAN_COUNT; i++) {
            SERIAL_PROTOCOL(job_recovery_info.fanSpeeds[i]);
            if (i < FAN_COUNT - 1) SERIAL_CHAR(',');
          }
          SERIAL_EOL();
        #endif

        //#if HAS_LEVELING
        //  SERIAL_PROTOCOLPAIR("leveling: ", int(job_recovery_info.leveling));
        //  SERIAL_PROTOCOLLNPAIR(" fade: ", int(job_recovery_info.fade));
        //#endif
        SERIAL_PROTOCOLLNPAIR("cmd_queue_index_r: ", int(job_recovery_info.cmd_queue_index_r));
        SERIAL_PROTOCOLLNPAIR("commands_in_queue: ", int(job_recovery_info.commands_in_queue));
        if (recovery)
          for (uint8_t i = 0; i < job_recovery_commands_count; i++) SERIAL_PROTOCOLLNPAIR("> ", job_recovery_commands[i]);
        else
          for (uint8_t i = 0; i < job_recovery_info.commands_in_queue; i++) SERIAL_PROTOCOLLNPAIR("> ", job_recovery_info.command_queue[i]);
        SERIAL_PROTOCOLLNPAIR("sd_filename: ", job_recovery_info.sd_filename);
        SERIAL_PROTOCOLLNPAIR("sdpos: ", job_recovery_info.sdpos);
        SERIAL_PROTOCOLLNPAIR("print_job_elapsed: ", job_recovery_info.print_job_elapsed);
      }
      else
        SERIAL_PROTOCOLLNPGM("INVALID DATA");
    }
    SERIAL_PROTOCOLLNPGM("---------------------------");
  }
#endif // DEBUG_POWER_LOSS_RECOVERY

/**
 * Check for Print Job Recovery during setup()
 *
 * If a saved state exists, populate job_recovery_commands with
 * commands to restore the machine state and continue the file.
 */
void check_print_job_recovery() {
  memset(&job_recovery_info, 0, sizeof(job_recovery_info));
  ZERO(job_recovery_commands);

  if (!card.cardOK) card.initsd();

  if (card.cardOK) {

    #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
      SERIAL_PROTOCOLLNPAIR("Init job recovery info. Size: ", int(sizeof(job_recovery_info)));
    #endif

    if (card.jobRecoverFileExists()) {
      card.openJobRecoveryFile(true);
      card.loadJobRecoveryInfo();
      card.closeJobRecoveryFile();

      if ((job_recovery_info.valid_head!=0)&&(job_recovery_info.valid_head == job_recovery_info.valid_foot)) {

	#ifdef LGT_MAC
		  check_recovery = true;
	#endif
        uint8_t ind = 0;
		feedrate_mm_s = job_recovery_info.feedrate;

		char str_Z[16], str_E[16];
		memset(str_Z, 0, sizeof(str_Z));
		memset(str_E, 0, sizeof(str_E));

		dtostrf(job_recovery_info.save_current_Z, 1, 3, str_Z); 
		dtostrf(job_recovery_info.save_current_E, 1, 3, str_E); 

	#ifdef U20_Pro
		sprintf_P(job_recovery_commands[ind++], PSTR("G28 R0 X0 Y0"));
		sprintf_P(job_recovery_commands[ind++], PSTR("M420 S0"));
		sprintf_P(job_recovery_commands[ind++], PSTR("M2007 E4"));
	#endif // U20_Pro
		sprintf_P(job_recovery_commands[ind++], PSTR("G92 Z%s E%s"), str_Z, str_E);
		sprintf_P(job_recovery_commands[ind++], PSTR("G28 R0 X0 Y0"));

        uint8_t r = job_recovery_info.cmd_queue_index_r, c = job_recovery_info.commands_in_queue;
        while (c--) {
          strcpy(job_recovery_commands[ind++], job_recovery_info.command_queue[r]);
          r = (r + 1) % BUFSIZE;
        }

        if (job_recovery_info.sd_filename[0] == '/') job_recovery_info.sd_filename[0] = ' ';
        sprintf_P(job_recovery_commands[ind++], PSTR("M23 %s"), job_recovery_info.sd_filename);
		sprintf_P(job_recovery_commands[ind++], PSTR("M24 S%ld"), job_recovery_info.sdpos);

        job_recovery_commands_count = ind;
        #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
          debug_print_job_recovery(true);
        #endif
      }
      else {
        if (job_recovery_info.valid_head != job_recovery_info.valid_foot)
          LCD_ALERTMESSAGEPGM("INVALID DATA");
        memset(&job_recovery_info, 0, sizeof(job_recovery_info));
      }
    }
  }
}

/**
 * Save the current machine state to the power-loss recovery file
 */
void save_job_recovery_info() {

  #if SAVE_INFO_INTERVAL_MS > 0
    static millis_t next_save_ms; // = 0;  // Init on reset
    millis_t ms = millis();
  #endif
  if ((current_position[2] > 0 &&abs(( current_position[2]+ recovery_z_height) -job_recovery_info.save_current_Z)>=0.1)) 
  {
	  int i = 0;
    #if SAVE_INFO_INTERVAL_MS > 0
      next_save_ms = ms + SAVE_INFO_INTERVAL_MS;
    #endif

    // Head and foot will match if valid data was saved
    if (!++job_recovery_info.valid_head) ++job_recovery_info.valid_head; // non-zero in sequence
    job_recovery_info.valid_foot = job_recovery_info.valid_head;

    // Machine state
	job_recovery_info.save_current_Z = current_position[2]+recovery_z_height;
	job_recovery_info.save_current_E = current_position[3];

    job_recovery_info.feedrate = feedrate_mm_s;
	job_recovery_info.have_percentdone = card.percentDone();

	for (i = 0; i < HOTENDS; i++)
	{
		job_recovery_info.target_temperature[i] = thermalManager.target_temperature[i];
	}
      job_recovery_info.target_temperature_bed = thermalManager.target_temperature_bed;

    #if FAN_COUNT
      COPY(job_recovery_info.fanSpeeds, fanSpeeds);
    #endif

    // Commands in the queue
    job_recovery_info.cmd_queue_index_r = cmd_queue_index_r;
    job_recovery_info.commands_in_queue = commands_in_queue;
    COPY(job_recovery_info.command_queue, command_queue);

    // Elapsed print job time
    job_recovery_info.print_job_elapsed = recovery_time+print_job_timer.duration();

    // SD file position
    card.getAbsFilename(job_recovery_info.sd_filename);
    job_recovery_info.sdpos = card.getIndex();

    #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
      SERIAL_PROTOCOLLNPGM("Saving...");
      debug_print_job_recovery(false);
    #endif

   card.openJobRecoveryFile();
    (void)card.saveJobRecoveryInfo();

    // If power-loss pin was triggered, write just once then kill
    #if PIN_EXISTS(POWER_LOSS)
      if (READ(POWER_LOSS_PIN) == POWER_LOSS_STATE) kill(MSG_POWER_LOSS_RECOVERY);
    #endif
  }
}

#endif // POWER_LOSS_RECOVERY
