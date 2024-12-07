#include "Commander.h"
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>


Commander::Commander(char eol, bool echo){
  this->eol = eol;
  this->echo = echo;
}


void Commander::add(char id, CommandCallback onCommand, const char* label ){
  call_list[call_count] = onCommand;
  call_ids[call_count] = id;
  call_label[call_count] = (char*)label;
  call_count++;
}

void Commander::run(char* user_input){
  // execute the user command
  char id = user_input[0];
  switch(id){
    case CMD_SCAN:
      for(int i=0; i < call_count; i++){
          printfMachineReadable(CMD_SCAN);
          printf(call_ids[i]);
          printf(":");
          if(call_label[i]) printf(call_label[i]);
          else printf("");
      }
      break;
    case CMD_VERBOSE:
      if(!isSentinel(user_input[1])) verbose = (VerboseMode)atoi(&user_input[1]);
      printfVerbose("Verb:");
      printfMachineReadable(CMD_VERBOSE);
      switch (verbose){
      case VerboseMode::nothing:
        printf("off!");
        break;
      case VerboseMode::on_request:
      case VerboseMode::user_friendly:
        printf("on!");
        break;
      case VerboseMode::machine_readable:
        printfMachineReadable("machine");
        break;
      }
      break;
    case CMD_DECIMAL:
      if(!isSentinel(user_input[1])) decimal_places = atoi(&user_input[1]);
      printfVerbose("Decimal:");
      printfMachineReadable(CMD_DECIMAL);
      printf(decimal_places);
      break;
    default:
      for(int i=0; i < call_count; i++){
        if(id == call_ids[i]){
          printfMachineReadable(user_input[0]);
          call_list[i](&user_input[1]);
          break;
        }
      }
      break;
  }
}

void Commander::motor(FOCMotor* motor, char* user_command) {

  // if target setting
  if(isDigit(user_command[0]) || user_command[0] == '-' || user_command[0] == '+' || isSentinel(user_command[0])){
    target(motor, user_command);
    return;
  }

  // parse command letter
  char cmd = user_command[0];
  char sub_cmd = user_command[1];
  // check if there is a subcommand or not
  int value_index = (sub_cmd >= 'A'  && sub_cmd <= 'Z') ||  (sub_cmd == '#') ?  2 :  1;
  // check if get command
  bool GET = isSentinel(user_command[value_index]);
  // parse command values
  float value = atof(&user_command[value_index]);
  printfMachineReadable(cmd);
  if (sub_cmd >= 'A'  && sub_cmd <= 'Z') {
    printfMachineReadable(sub_cmd);
  }

  // a bit of optimisation of variable memory for Arduino UNO (atmega328)
  switch(cmd){
    case CMD_C_Q_PID:      //
      printfVerbose("PID curr q| ");
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_q, &user_command[1]);
      else pid(&motor->PID_current_q,&user_command[1]);
      break;
    case CMD_C_D_PID:      //
      printfVerbose("PID curr d| ");
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_d, &user_command[1]);
      else pid(&motor->PID_current_d, &user_command[1]);
      break;
    case CMD_V_PID:      //
      printfVerbose("PID vel| ");
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_velocity, &user_command[1]);
      else pid(&motor->PID_velocity, &user_command[1]);
      break;
    case CMD_A_PID:      //
      printfVerbose("PID angle| ");
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_angle, &user_command[1]);
      else pid(&motor->P_angle, &user_command[1]);
      break;
    case CMD_LIMITS:      //
     printfVerbose("Limits| ");
      switch (sub_cmd){
        case SCMD_LIM_VOLT:      // voltage limit change
          printfVerbose("volt: ");
          if(!GET) {
            motor->voltage_limit = value;
            motor->PID_current_d.limit = value;
            motor->PID_current_q.limit = value;
            // change velocity pid limit if in voltage mode and no phase resistance set
            if( !_isset(motor->phase_resistance) && motor->torque_controller==TorqueControlType::voltage) motor->PID_velocity.limit = value;
          }
          printf("%f", motor->voltage_limit);
          break;
        case SCMD_LIM_CURR:      // current limit
          printfVerbose("curr: ");
          if(!GET){
            motor->current_limit = value;
            // if phase resistance specified or the current control is on set the current limit to the velocity PID
            if(_isset(motor->phase_resistance) || motor->torque_controller != TorqueControlType::voltage ) motor->PID_velocity.limit = value;
          }
          printf("%f", motor->current_limit);
          break;
        case SCMD_LIM_VEL:      // velocity limit
          printfVerbose("vel: ");
          if(!GET){
            motor->velocity_limit = value;
            motor->P_angle.limit = value;
          }
          printf("%f", motor->velocity_limit);
          break;
        default:
          printfError();
          break;
      }
      break;
    case CMD_MOTION_TYPE:
    case CMD_TORQUE_TYPE:
    case CMD_STATUS:
      motion(motor, &user_command[0]);
      break;
    case CMD_PWMMOD:
       // PWM modulation change
       printfVerbose("PWM Mod | ");
       switch (sub_cmd){
        case SCMD_PWMMOD_TYPE:      // zero offset
          printfVerbose("type: ");
          if(!GET) motor->foc_modulation = (FOCModulationType)value;
          switch(motor->foc_modulation){
            case FOCModulationType::SinePWM:
              printf("SinePWM");
              break;
            case FOCModulationType::SpaceVectorPWM:
              printf("SVPWM");
              break;
            case FOCModulationType::Trapezoid_120:
              printf("Trap 120");
              break;
            case FOCModulationType::Trapezoid_150:
              printf("Trap 150");
              break;
          }
          break;
        case SCMD_PWMMOD_CENTER:      // centered modulation
          printfVerbose("center: ");
          if(!GET) motor->modulation_centered = value;
          printf(motor->modulation_centered);
          break;
        default:
          printfError();
          break;
       }
      break;
    case CMD_RESIST:
      printfVerbose("R phase: ");
      if(!GET){
        motor->phase_resistance = value;
        if(motor->torque_controller==TorqueControlType::voltage)
          motor->PID_velocity.limit= motor->current_limit;
      }
      if(_isset(motor->phase_resistance)) printf("%f", motor->phase_resistance);
      else printf(0);
      break;
    case CMD_INDUCTANCE:
      printfVerbose("L phase: ");
      if(!GET){
        motor->phase_inductance = value;
      }
      if(_isset(motor->phase_inductance)) printf("%f", motor->phase_inductance);
      else printf("%d", 0);
      break;
    case CMD_KV_RATING:
      printfVerbose("Motor KV: ");
      if(!GET){
        motor->KV_rating = value;
      }
      if(_isset(motor->KV_rating)) printf("%f", motor->KV_rating);
      else printf(0);
      break;
    case CMD_SENSOR:
      // Sensor zero offset
       printfVerbose("Sensor | ");
       switch (sub_cmd){
        case SCMD_SENS_MECH_OFFSET:      // zero offset
          printfVerbose("offset: ");
          if(!GET) motor->sensor_offset = value;
          printf("%f", motor->sensor_offset);
          break;
        case SCMD_SENS_ELEC_OFFSET:      // electrical zero offset - not suggested to touch
          printfVerbose("el. offset: ");
          if(!GET) motor->zero_electric_angle = value;
          printf("%f", motor->zero_electric_angle);
          break;
        default:
          printfError();
          break;
       }
      break;
    case CMD_MONITOR:     // get current values of the state variables
      printfVerbose("Monitor | ");
      switch (sub_cmd){
        case SCMD_GET:      // get command
          switch((uint8_t)value){
            case 0: // get target
              printfVerbose("target: ");
              printf("%f", motor->target);
              break;
            case 1: // get voltage q
              printfVerbose("Vq: ");
              printf("%f", motor->voltage.q);
              break;
            case 2: // get voltage d
              printfVerbose("Vd: ");
              printf("%f", motor->voltage.d);
              break;
            case 3: // get current q
              printfVerbose("Cq: ");
              printf("%f", motor->current.q);
              break;
            case 4: // get current d
              printfVerbose("Cd: ");
              printf("%f", motor->current.d);
              break;
            case 5: // get velocity
              printfVerbose("vel: ");
              printf("%f", motor->shaft_velocity);
              break;
            case 6: // get angle
              printfVerbose("angle: ");
              printf("%f", motor->shaft_angle);
              break;
            case 7: // get all states
              printfVerbose("all: ");
              printf("%f", motor->target);
              printf(";");
              printf("%f", motor->voltage.q);
              printf(";");
              printf("%f", motor->voltage.d);
              printf(";");
              printf("%f", motor->current.q);
              printf(";");
              printf("%f", motor->current.d);
              printf(";");
              printf("%f", motor->shaft_velocity);
              printf(";");
              printf("%f", motor->shaft_angle);
              break;
            default:
              printfError();
              break;
          }
          break;
        case SCMD_DOWNSAMPLE:
          printfVerbose("downsample: ");
          if(!GET) motor->monitor_downsample = value;
          printf((int)motor->monitor_downsample);
          break;
        case SCMD_CLEAR:
          motor->monitor_variables = (uint8_t) 0;
          printf("clear");
          break;
        case CMD_DECIMAL:
          printfVerbose("decimal: ");
          motor->monitor_decimals = value;
          printf((int)motor->monitor_decimals);
          break;
        case SCMD_SET:
          if(!GET){
            // set the variables
            motor->monitor_variables = (uint8_t) 0;
            for(int i = 0; i < 7; i++){
              if(isSentinel(user_command[value_index+i])) break;
              motor->monitor_variables |=  (user_command[value_index+i] - '0') << (6-i);
            }
          }
          // printf the variables
          for(int i = 0; i < 7; i++){
            printf( (motor->monitor_variables & (1 << (6-i))) >> (6-i));
          }
          printf("");
          break;
        default:
          printfError();
          break;
       }
      break;
    default:  // unknown cmd
      printfVerbose("unknown cmd ");
      printfError();
  }
}

void Commander::motion(FOCMotor* motor, char* user_cmd, char* separator){
  char cmd = user_cmd[0];
  char sub_cmd = user_cmd[1];
  bool GET  = isSentinel(user_cmd[1]);
  float value = atof(&user_cmd[(sub_cmd >= 'A'  && sub_cmd <= 'Z') ?  2 :  1]);

  switch(cmd){
    case CMD_MOTION_TYPE:
      printfVerbose("Motion:");
      switch(sub_cmd){
        case SCMD_DOWNSAMPLE:
            printfVerbose(" downsample: ");
            if(!GET) motor->motion_downsample = value;
            printf((int)motor->motion_downsample);
          break;
        default:
          // change control type
          if(!GET && value >= 0 && (int)value < 5) // if set command
            motor->controller = (MotionControlType)value;
          switch(motor->controller){
            case MotionControlType::torque:
              printf("torque");
              break;
            case MotionControlType::velocity:
              printf("vel");
              break;
            case MotionControlType::angle:
              printf("angle");
              break;
            case MotionControlType::velocity_openloop:
              printf("vel open");
              break;
            case MotionControlType::angle_openloop:
              printf("angle open");
              break;
          }
            break;
        }
      break;
    case CMD_TORQUE_TYPE:
      // change control type
      printfVerbose("Torque: ");
      if(!GET && (int8_t)value >= 0 && (int8_t)value < 3)// if set command
        motor->torque_controller = (TorqueControlType)value;
      switch(motor->torque_controller){
        case TorqueControlType::voltage:
          printf("volt");
          // change the velocity control limits if necessary
          if( !_isset(motor->phase_resistance) ) motor->PID_velocity.limit = motor->voltage_limit;
          break;
        case TorqueControlType::dc_current:
          printf("dc curr");
          // change the velocity control limits if necessary
          motor->PID_velocity.limit = motor->current_limit;
          break;
        case TorqueControlType::foc_current:
          printf("foc curr");
          // change the velocity control limits if necessary
          motor->PID_velocity.limit = motor->current_limit;
          break;
      }
      break;
    case CMD_STATUS:
      // enable/disable
      printfVerbose("Status: ");
      if(!GET) (bool)value ? motor->enable() : motor->disable();
       printf(motor->enabled);
      break;
    default:
      target(motor,  user_cmd, separator);
      break;
  }
}

void Commander::pid(PIDController* pid, char* user_cmd){
  char cmd = user_cmd[0];
  bool GET  = isSentinel(user_cmd[1]);
  float value = atof(&user_cmd[1]);

  switch (cmd){
    case SCMD_PID_P:      // P gain change
      printfVerbose("P: ");
      if(!GET) pid->P = value;
      printf("%f", pid->P);
      break;
    case SCMD_PID_I:      // I gain change
      printfVerbose("I: ");
      if(!GET) pid->I = value;
      printf("%f", pid->I);
      break;
    case SCMD_PID_D:      // D gain change
      printfVerbose("D: ");
      if(!GET) pid->D = value;
      printf("%f", pid->D);
      break;
    case SCMD_PID_RAMP:      //  ramp change
      printfVerbose("ramp: ");
      if(!GET) pid->output_ramp = value;
      printf("%f", pid->output_ramp);
      break;
    case SCMD_PID_LIM:      //  limit change
      printfVerbose("limit: ");
      if(!GET) pid->limit = value;
      printf("%f", pid->limit);
      break;
    default:
      printfError();
      break;
  }
}

void Commander::lpf(LowPassFilter* lpf, char* user_cmd){
  char cmd = user_cmd[0];
  bool GET  = isSentinel(user_cmd[1]);
  float value = atof(&user_cmd[1]);

  switch (cmd){
    case SCMD_LPF_TF:      // Tf value change
      printfVerbose("Tf: ");
      if(!GET) lpf->Tf = value;
      printf("%f", lpf->Tf);
      break;
    default:
      printfError();
      break;
  }
}

void Commander::scalar(float* value,  char* user_cmd){
  bool GET  = isSentinel(user_cmd[0]);
  if(!GET) *value = atof(user_cmd);
  printf("%f", *value);
}


void Commander::target(FOCMotor* motor,  char* user_cmd, char* separator){
  // if no values sent
  if(isSentinel(user_cmd[0])) {
    printfMachineReadable(motor->target);
    return;
  };

  float pos, vel, torque;
  char* next_value;
  switch(motor->controller){
    case MotionControlType::torque: // setting torque target
      torque = atof(strtok (user_cmd, separator));
      motor->target = torque;
      break;
    case MotionControlType::velocity: // setting velocity target + torque limit
      // set the target
      vel= atof(strtok (user_cmd, separator));
      motor->target = vel;

      // allow for setting only the target velocity without chaning the torque limit
      next_value = strtok (NULL, separator);
      if (next_value){
        torque = atof(next_value);
        motor->PID_velocity.limit = torque;
        // torque command can be voltage or current
        if(!_isset(motor->phase_resistance) && motor->torque_controller == TorqueControlType::voltage) motor->voltage_limit = torque;
        else  motor->current_limit = torque;
      }
      break;
    case MotionControlType::angle: // setting angle target + torque, velocity limit
      // setting the target position
      pos= atof(strtok (user_cmd, separator));
      motor->target = pos;

      // allow for setting only the target position without chaning the velocity/torque limits 
      next_value = strtok (NULL, separator);
      if( next_value ){
        vel = atof(next_value);
        motor->velocity_limit = vel;
        motor->P_angle.limit = vel;
  
        // allow for setting only the target position and velocity limit without the torque limit 
        next_value = strtok (NULL, separator);
        if( next_value ){
          torque= atof(next_value);
          motor->PID_velocity.limit = torque;
          // torque command can be voltage or current
          if(!_isset(motor->phase_resistance) && motor->torque_controller == TorqueControlType::voltage) motor->voltage_limit = torque;
          else  motor->current_limit = torque;
        }
      }
      break;
    case MotionControlType::velocity_openloop: // setting velocity target + torque limit
      // set the target
      vel= atof(strtok (user_cmd, separator));
      motor->target = vel;
      // allow for setting only the target velocity without chaning the torque limit
      next_value = strtok (NULL, separator);
      if (next_value ){
        torque = atof(next_value);
        // torque command can be voltage or current
        if(!_isset(motor->phase_resistance)) motor->voltage_limit = torque;
        else  motor->current_limit = torque;
      }
      break;
    case MotionControlType::angle_openloop: // setting angle target + torque, velocity limit
      // set the target
      pos= atof(strtok (user_cmd, separator));
      motor->target = pos; 
      
      // allow for setting only the target position without chaning the velocity/torque limits 
      next_value = strtok (NULL, separator);
      if( next_value ){
        vel = atof(next_value);
        motor->velocity_limit = vel;
        // allow for setting only the target velocity without chaning the torque limit
        next_value = strtok (NULL, separator);
        if (next_value ){
          torque = atof(next_value);
          // torque command can be voltage or current
          if(!_isset(motor->phase_resistance)) motor->voltage_limit = torque;
          else  motor->current_limit = torque;
        }
      }
      break;
  }
  printfVerbose("Target: ");
  printf("%f", motor->target);
}


bool Commander::isSentinel(char ch)
{
  if(ch == eol)
    return true;
  else if (ch == '\r')
  {
      printfVerbose("Warn: \\r detected! \n");
      return true; // lets still consider it to end the line...
  }
  return false;
}

void Commander::printVerbose(const char* message){
  if(verbose == VerboseMode::user_friendly) printf(message);
}

void Commander::printMachineReadable(const int number){
  if(verbose == VerboseMode::machine_readable) printf(number);
}
void Commander::printMachineReadable(const float number){
  if(verbose == VerboseMode::machine_readable) printf(number);
}
void Commander::printMachineReadable(const char* message){
  if(verbose == VerboseMode::machine_readable) printf(message);
}
void Commander::printMachineReadable(const char message){
  if(verbose == VerboseMode::machine_readable) printf(message);
}

void Commander::printlnMachineReadable(const int number){
  if(verbose == VerboseMode::machine_readable) printf(number);
}
void Commander::printlnMachineReadable(const float number){
  if(verbose == VerboseMode::machine_readable) printf(number);
}
void Commander::printlnMachineReadable(const char* message){
  if(verbose == VerboseMode::machine_readable) printf(message);
}
void Commander::printlnMachineReadable(const char message){
  if(verbose == VerboseMode::machine_readable) printf(message);
}

void Commander::printfError(){
 printf("err");
}
