/* ********************************************************************
FILE                   : selective_two_fsm.c

PROGRAM DESCRIPTION    : elevator control is based on two basic principles.
  1: Continue to travel in the current elevator movement direction(up or down) while there are still remaining requests in that same elevator movement direction.
  2: If there are no further requests in that direction, then stop and become idle, or change direction if there are requests in the opposite direction.
  elevator's in cabin floor call for each floor and only one hall floor call for each floor are considered. 
  So when either in cabin floor call or hall floor call is active, then floor call for that floor is active.
  We simulate, floor call as request for that floor and implemented elevator control principles 

AUTHOR                :  K.M. Arun Kumar alias Arunkumar Murugeswaran
	 
KNOWN BUGS            :    

NOTE                  :  Compiled and Tested in Dev-C++ on Windows 7 (32 bit) Desktop OS.
                                    
CHANGE LOGS           : 

*****************************************************************************/
#include <stdio.h>
#define SUCCESS         (0) 
#define FAILURE         (1)

typedef enum 
{
	NO_ERROR, ERR_NULL_PTR,ERR_FORMAT_INVALID , ERR_QUEUE_FULL, ERR_QUEUE_EMPTY, ERR_ENQUEUE_PROC, ERR_DEQUEUE_PROC, ERR_QUEUE_INSERT_FRONT_PROC, ERR_QUEUE_DELETE_REAR_PROC,
	ERR_QUEUE_RETRIEVE_INFO_PROC, ERR_DISP_QUEUE_PROC, ERR_INSERT_DATA_PROC, ERR_RETRIEVE_DATA_PROC, NUM_SYS_STATUS
} system_status_t;

typedef enum
{
	ERR_UART_NOT_DATA_SRC = NUM_SYS_STATUS, ERR_FLOOR_INVALID, ERR_ELEVATOR_CH_ID_EXCEEDS, ERR_ELEVATOR_DISP_STATUS, ERR_ELEVATOR_FSM_PROC, 
	ERR_ELEVATOR_FSM_INVALID, ERR_ELEVATOR_BEFORE_FSM_INVALID, ERR_ELEVATOR_DISP_PROC, ERR_ELEVATOR_CUR_FSM_INVALID, ERR_CAR_MOVE_STATE_PROC, 
	ERR_DELAY_TIME_ELEVATOR_PROC, ERR_REQ_CONDITION_PROC, ERR_POLL_FLOOR_CALLS_PROC, ERR_CAR_UNMOVED, ERR_NEXT_FLOOR_INVALID, ERR_NEXT_STOP_FLOOR_PROC, 
	ERR_STOP_NOT_CUR_FLOOR, ERR_MORE_THAN_ONE_SW_DETECT, ERR_STARTUP_OPER, ERR_CHECK_MOVE, ERR_SW_IS_INACTIVE,  ERR_SW_IS_ACTIVE, ERR_SW_IS_DISABLED,
	ERR_STOP_FLOOR_NOT_LIST, ERR_NEXT_FLOOR_NOT_DETECT, ERR_CHECK_STATIONARY, ERR_OVERLOAD_NO_USER, ERR_NO_PENDING_CALL_DOOR_CLOSED, 
	ERR_COMPUTE_STOP_FLOOR_DATAS, WARN_FLOORS_CALL_PROC, WARN_IN_CAR_FLOOR_CALL_IS_ACTIVE, WARN_HALL_CALL_UP_FLOOR_IS_ACTIVE, WARN_HALL_CALL_DOWN_FLOOR_IS_ACTIVE, 
	ERR_HALL_CALL_UP_FLOOR_INVALID, ERR_HALL_CALL_DOWN_FLOOR_INVALID, ERR_FLOOR_CALLS_NOT_ALLOWED, WARN_FLOOR_CALL_IS_ACTIVE, ERR_NUM_APPL_STATUS
} appl_status_t;

typedef enum 
{
	MOVE_UP, MOVE_DOWN, STARTUP_STATIONARY, MOVED_UP_STATIONARY, MOVED_DOWN_STATIONARY,  NO_PENDING_FLOOR_CALLS, 
	TRIGGER_MOVE_UP_NO_DIR_CHANGE, TRIGGER_MOVE_UP_DIR_CHANGE, TRIGGER_MOVE_DOWN_NO_DIR_CHANGE, TRIGGER_MOVE_DOWN_DIR_CHANGE, 
	TRIGGERED_EMERGENCY_STOP, ERR_MOVE_UP_NEXT_FLOOR_INVALID, ONE_PENDING_FLOOR_CALL, ERR_MOVE_DOWN_NEXT_FLOOR_INVALID, ERR_DOOR_OPEN_NOT_DETECT, 
	ERR_DOOR_CLOSE_NOT_DETECT, ERR_REQ_DOORS_ALIGN_BUT_UNALIGN, ERR_AT_LEAST_TWO_LIMIT_SW_DETECT, ERR_STARTUP_AT_LEAST_TWO_LIMIT_SW_DETECT, 
	ERR_MOVING_UP_NEXT_FLOOR_NOT_DETECT, ERR_MOVING_DOWN_NEXT_FLOOR_NOT_DETECT, ERR_DOORS_NOT_ALIGNED_PROPERLY, ERR_REQ_DOOR_CLOSE_BUT_OPENED, 
	ERR_REQ_DOOR_OPEN_BUT_CLOSED, ERR_MOVE_UP_BUT_UNMOVED, ERR_MOVE_DOWN_BUT_UNMOVED, ERR_STARTUP_CUR_FLOOR_NOT_DETECT, ERR_DOOR_OPEN_AND_CLOSE_ACTIVE_FOR_OPEN, 
	ERR_DOOR_OPEN_AND_CLOSE_ACTIVE_FOR_CLOSE, ERR_DOOR_OPEN_FAST, ERR_DOOR_CLOSE_FAST, ERR_REQ_DOOR_CLOSED_BUT_NOT_CLOSED, ERR_REQ_DOORS_UNALIGN_BUT_ALIGN, 
	ERR_DOOR_OPENED_BUT_READY_MOVE, ERR_DOOR_NOT_CLOSED_BUT_READY_MOVE, ERR_STARTUP_DOOR_OPEN_AND_CLOSE_ACTIVE, ERR_DOORS_UNALIGNED_BUT_OPENED,
	ERR_DOORS_UNALIGNED_BUT_NOT_CLOSED, ERR_DOORS_ALIGNED_FOR_MOVE, ERR_DOORS_UNALIGNED_AT_STATIONARY, ERR_DOOR_NOT_OPENED_AT_STATIONARY,
	ERR_DOORS_CLOSED_AT_STATIONARY, ERR_LIMIT_FLOOR_NOT_DETECT_AT_STATIONARY, ERR_OVERLOAD_BUT_NO_IN_CAR_USER_PRESENT, 
	ERR_NEXT_STOP_FLOOR_REACH_BUT_NOT_IN_STOP_LIST, ERR_MAX_STOP_FLOOR_REACH_BUT_NOT_IN_STOP_LIST, ERR_MIN_STOP_FLOOR_REACH_BUT_NOT_IN_STOP_LIST,
	ERR_NEXT_STOP_MORE_THAN_MAX_STOP_FLOOR, ERR_NEXT_STOP_LESS_THAN_MIN_STOP_FLOOR, ERR_COMPUTE_NEXT_FLOOR_STOP_PROC, ERR_ELEVATOR_NOT_STATIONARY, 
	ERR_MIN_STOP_FLOOR_INVALID, ERR_MIN_STOP_FLOOR_NOT_IN_STOP_LIST, ERR_MAX_STOP_FLOOR_INVALID, WARN_FLOOR_CALL_ALREADY_SELECT, WARN_FLOOR_CALL_SAME_CUR_FLOOR,
	ERR_COMPUTE_NEXT_STOP_FLOOR, ERR_NEXT_STOP_FLOOR_NOT_STOPPED, ERR_INTERNAL_PROC_ABNORMAL, ERR_CUR_FLOOR_INVALID, ERR_POLL_HALL_AND_IN_CAR_PROC, 
	ERR_DELAY_TIME_PROC, ERR_CAR_MOVEMENT_PROC, ERR_DISP_STATUS, NUM_ELEVATOR_STATUS
} elevator_status_t;

typedef enum
{
	FSM_IDLE, FSM_WAIT_FOR_START_OPER, FSM_STARTUP, FSM_DECIDE_CAR_MOVEMENT, FSM_TRIGGER_MOVE_UP, FSM_TRIGGER_MOVE_DOWN, 
	FSM_PREPARE_TO_MOVE, FSM_MOVING, FSM_TRIGGER_CAR_STOP, FSM_WAIT_TILL_CAR_STOPPED, FSM_PREPARE_DOORS_TO_ALIGN, 
	FSM_WAIT_TILL_DOORS_ALIGN, FSM_TRIGGER_DOOR_OPEN, FSM_WAIT_TILL_DOOR_START_OPEN, FSM_WAIT_TILL_DOOR_OPENED, 
	FSM_PREPARE_USER_ENTRY_AND_EXIT, FSM_USER_ENTRY_AND_EXIT, FSM_PREPARE_FOR_DOOR_CLOSE, FSM_TRIGGER_DOOR_CLOSE, 
	FSM_WAIT_TILL_DOOR_START_CLOSE, FSM_WAIT_TILL_DOOR_CLOSED, FSM_WAIT_TILL_DOORS_TO_UNALIGN, FSM_COMPUTE_NEXT_STOP_FLOOR,
    FSM_WAIT_FOR_DOOR_CLOSE_TO_START_OPEN, FSM_ABNORMAL_EVENT, NUM_ELEVATOR_FSM_STATES  	
} elevator_oper_fsm_states_t;


#define ERROR_OCCURED    (1)
#define WARNING_OCCURED  (2) 
#define NULL_PTR                          ((void *) 0)
#define DATA_NA                           (0)

typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long uint64_t;

typedef char int8_t;
typedef short int int16_t;
typedef int int32_t;
typedef long int64_t;

uint32_t system_status_flag = NO_ERROR;
uint32_t appl_status_flag = NO_ERROR;

typedef enum
{
	RESET_WHOLE, RESET_DATA_IDS_AND_APPL, RESET_APPL
} reset_status_t;

uint8_t Error_or_Warning_Proc(const char *const error_trace_str, const uint8_t warn_or_error_format, const uint32_t warning_or_error_code);
uint16_t Appl_Reset(const uint8_t reset_type);
uint16_t Appl_Reset_Proc(const uint8_t cur_ctrl_elevator_ch_id);
uint16_t Reset_Elevator_Datas(const uint8_t ctrl_elevator_ch_id, const uint8_t reset_type);
uint16_t Validate_Floor(const uint8_t floor);
uint16_t Compute_Next_Floor_Stop(const uint8_t cur_ctrl_elevator_ch_id, uint8_t *const elevator_next_fsm_state_ptr);
uint16_t FSM_StartUp_Proc(const uint8_t ctrl_elevator_ch_id);
uint16_t FSM_Decide_Car_Move_Proc(const uint8_t ctrl_elevator_ch_id);
uint16_t FSM_Car_Moving_Proc(const uint8_t ctrl_elevator_ch_id);
uint16_t FSM_User_Entry_And_Exit_Proc(const uint8_t ctrl_elevator_ch_id);
uint16_t FSM_Compute_Next_Stop_Floor_Proc(const uint8_t ctrl_elevator_ch_id);
uint16_t FSM_Abnormal_Event_Proc(const uint8_t ctrl_elevator_ch_id);
uint16_t Active_Hall_And_In_Car_Calls_Proc(const uint8_t ctrl_elevator_ch_id);
uint16_t Disp_Oper_Proc(const uint8_t ctrl_elevator_ch_id);
uint16_t Car_Movement_Direction(const uint8_t ctrl_elevator_ch_id, const uint8_t cur_floor, uint8_t *const elevator_trigger_move_ptr);
uint16_t Compute_Floor_Stop_Datas(const uint8_t ctrl_elevator_ch_id, const uint8_t stage_type, uint8_t *const elevator_status_ptr);
	
#define DEFAULT_FLOOR           (0)
#define MIN_NUM_FLOORS          (3)
#define MAX_NUM_FLOORS         (32)  

#define MAX_NUM_ELEVATORS       (1)
#define CTRL_ELEVATOR_CH_ID     (0)

//#define TRACE                                 (1U)
#define TRACE_ERROR                             (2U)
#define TRACE_REQ                               (3U)
#define TRACE_INFO                              (4U)
//#define TRACE_DATA                            (5U)
#define TRACE_FLOW                            (6U)

typedef enum 
{ 
  DOOR_CLOSED_STAGE, DOOR_OPENED_STAGE, NUM_ELEVATOR_STAGES 
} elevator_stages_t;

typedef enum 
{
	FLOOR_00, FLOOR_01, FLOOR_02, FLOOR_03, FLOOR_04
} floor_id_t;

typedef struct
{
   uint8_t cur_max_floor_call;
   uint8_t cur_min_floor_call;
   uint8_t cur_max_floor_call_up;
   uint8_t cur_min_floor_call_up;
   uint8_t cur_max_floor_call_down;
   uint8_t cur_min_floor_call_down;
   uint8_t cur_max_floor_call_in_car;
   uint8_t cur_min_floor_call_in_car;
   uint8_t cur_floor;
   uint8_t elevator_status;  
   uint8_t next_stop_floor;
   uint8_t cur_fsm_state;
   uint8_t before_fsm_state;
   uint32_t pending_floor_call_bit_field;
   uint32_t in_car_floor_call_bit_field;
   uint32_t hall_floor_call_up_bit_field;
   uint32_t hall_floor_call_down_bit_field;
   uint8_t num_pending_calls;
   uint8_t num_pending_calls_in_car;
   uint8_t num_pending_calls_up;
   uint8_t num_pending_calls_down;
  
} elevator_ctrl_and_status_t;

elevator_ctrl_and_status_t elevator_ctrl_and_status[MAX_NUM_ELEVATORS];

typedef enum
{
	RESET_STOP_DATAS, RESET_WHOLE_DATAS
} elevator_reset_type_t;

typedef enum
{
	FSM_OPER = 1, FLOOR_CALL_OPER, DISP_OPER, RESET_OPER, EXIT_OPER, IN_CAR_FLOOR_CALL_OPER = 1, HALL_UP_FLOOR_CALL_OPER, HALL_DOWN_FLOOR_CALL_OPER 
} choice_oper_t;

uint32_t uint32_temp_disp_data = 0;
uint8_t max_num_floors = 0;

/*------------------------------------------------------------*
FUNCTION NAME  : main

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 02.13  

BUGS           :              
-*------------------------------------------------------------*/
int main()
{
	elevator_ctrl_and_status_t *cur_elevator_ptr;
	uint16_t ret_status;
	int8_t choice; 
		
	cur_elevator_ptr = elevator_ctrl_and_status + CTRL_ELEVATOR_CH_ID;	
	if((ret_status = Appl_Reset(RESET_APPL)) != SUCCESS)
	{
		return FAILURE;
	}
	while(1)
	{
		printf("MENU : 1 - FSM , 2 - Floor call, 3 - Disp, 4 - Reset, 5 - Exit\n");
		printf("Enter choice : ");
		scanf("%d", &choice);
		switch(choice)
		{
			case FSM_OPER:
		        switch(cur_elevator_ptr->cur_fsm_state)
		        {
		           case FSM_STARTUP:			
			          FSM_StartUp_Proc(CTRL_ELEVATOR_CH_ID);			          
                   break;
                   case FSM_DECIDE_CAR_MOVEMENT:			 
			          FSM_Decide_Car_Move_Proc(CTRL_ELEVATOR_CH_ID);
                   break;
			       case FSM_MOVING:
			          FSM_Car_Moving_Proc(CTRL_ELEVATOR_CH_ID);				      
                   break;
			       case FSM_USER_ENTRY_AND_EXIT:
			          FSM_User_Entry_And_Exit_Proc(CTRL_ELEVATOR_CH_ID);				      
			       break;
			       case FSM_COMPUTE_NEXT_STOP_FLOOR:
			          FSM_Compute_Next_Stop_Floor_Proc(CTRL_ELEVATOR_CH_ID);
                   break;
			       case FSM_ABNORMAL_EVENT:
                       FSM_Abnormal_Event_Proc(CTRL_ELEVATOR_CH_ID);			       
			       break;	
		        }			 
            break;  
			case FLOOR_CALL_OPER:
			    Active_Hall_And_In_Car_Calls_Proc(CTRL_ELEVATOR_CH_ID);  
			break;
			case DISP_OPER:
			   Disp_Oper_Proc(CTRL_ELEVATOR_CH_ID);
			break;
			case RESET_OPER:
			  Appl_Reset(RESET_APPL);
			break;
			case EXIT_OPER:
			  return SUCCESS;
			//break;
			default:
			  printf("Invalid oper \n");
		}
	}
    return SUCCESS;
}

/*------------------------------------------------------------*
FUNCTION NAME  : FSM_StartUp_Proc

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 14.03  

BUGS           :              
-*------------------------------------------------------------*/
uint16_t FSM_StartUp_Proc(const uint8_t ctrl_elevator_ch_id)
{
	elevator_ctrl_and_status_t *cur_elevator_ptr;
	uint16_t ret_status;
	uint8_t cur_floor;
	
	cur_elevator_ptr = elevator_ctrl_and_status + ctrl_elevator_ch_id;
	printf("Enter car startup floor : ");
	scanf("%u", &cur_floor);
	if((ret_status = Validate_Floor(cur_floor)) != SUCCESS)
	{
	   Error_or_Warning_Proc("14.04.02", ERROR_OCCURED, appl_status_flag);
	   return FAILURE;		  
	}
	cur_elevator_ptr->cur_floor = cur_floor;
	cur_elevator_ptr->next_stop_floor = DEFAULT_FLOOR;
	#ifdef TRACE_FLOW
	    	printf("STARTUP -> USER_ENTRY_AND_EXIT \n");
	#endif
	cur_elevator_ptr->cur_fsm_state = FSM_USER_ENTRY_AND_EXIT;
    cur_elevator_ptr->elevator_status = STARTUP_STATIONARY;  
	return SUCCESS;
}      

/*------------------------------------------------------------*
FUNCTION NAME  : FSM_Decide_Car_Move_Proc

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 14.04 

BUGS           :              
-*------------------------------------------------------------*/
uint16_t FSM_Decide_Car_Move_Proc(const uint8_t ctrl_elevator_ch_id)
{
	elevator_ctrl_and_status_t *cur_elevator_ptr;	
	uint16_t ret_status;
	uint8_t car_movement_state;
	
	if(ctrl_elevator_ch_id >= MAX_NUM_ELEVATORS)
	{
		appl_status_flag = ERR_ELEVATOR_CH_ID_EXCEEDS;
	    Error_or_Warning_Proc("14.04.01", ERROR_OCCURED, appl_status_flag);
	    return appl_status_flag;
    }	
    cur_elevator_ptr = elevator_ctrl_and_status + ctrl_elevator_ch_id;
	if((ret_status = Car_Movement_Direction(ctrl_elevator_ch_id, cur_elevator_ptr->cur_floor, &car_movement_state)) != SUCCESS)
	{
	    appl_status_flag = ERR_CAR_MOVE_STATE_PROC;
	    Error_or_Warning_Proc("14.04.03", ERROR_OCCURED, appl_status_flag);
	    #ifdef TRACE_ERROR 
	       printf("DECIDE_CAR_MOVEMENT -> ABNORMAL_EVENT \n");
		   printf("ERR: event - Car movement proc \n");
	    #endif
		cur_elevator_ptr->elevator_status = ERR_CAR_MOVEMENT_PROC;
	    cur_elevator_ptr->cur_fsm_state = FSM_ABNORMAL_EVENT;
		return SUCCESS;	 
	}
	switch(car_movement_state)
	{
		   case STARTUP_STATIONARY:
		       #ifdef TRACE_FLOW
	              printf("DECIDE_CAR_MOVEMENT -> USER_ENTRY_AND_EXIT \n");	                  
	           #endif			   
			   cur_elevator_ptr->cur_fsm_state =  FSM_USER_ENTRY_AND_EXIT;			 
		   break;
		   case MOVED_DOWN_STATIONARY:
		   case MOVED_UP_STATIONARY:
		       #ifdef TRACE_FLOW
	              printf("DECIDE_CAR_MOVEMENT -> USER_ENTRY_AND_EXIT \n");	                  
	           #endif
		       cur_elevator_ptr->cur_fsm_state = FSM_USER_ENTRY_AND_EXIT ;			  
		   break;
           case MOVE_UP:
		       #ifdef TRACE_FLOW
	              printf("DECIDE_CAR_MOVEMENT -> MOVE_UP\n");	                  
	           #endif 
			   #ifdef TRACE_INFO
			       printf("INFO: Current floor : %u, next stop floor: %u\n", cur_elevator_ptr->cur_floor, cur_elevator_ptr->next_stop_floor);
			   #endif
               cur_elevator_ptr->cur_fsm_state =  FSM_MOVING;			  
           break;
           case MOVE_DOWN:
		       #ifdef TRACE_FLOW
	              printf("DECIDE_CAR_MOVEMENT -> MOVE_DOWN\n");	                  
	           #endif 
			   #ifdef TRACE_INFO
			       printf("INFO: Current floor : %u, next stop floor: %u\n", cur_elevator_ptr->cur_floor, cur_elevator_ptr->next_stop_floor);
			   #endif
               cur_elevator_ptr->cur_fsm_state =  FSM_MOVING;			  
           break;
		   default:
              appl_status_flag = ERR_FORMAT_INVALID;
	          Error_or_Warning_Proc("14.04.04", ERROR_OCCURED, appl_status_flag);
	          return appl_status_flag;	
	 }
	 cur_elevator_ptr->elevator_status = car_movement_state;	
     return SUCCESS;
}

/*------------------------------------------------------------*
FUNCTION NAME  : FSM_Car_Moving_Proc 

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 14.08 

BUGS           :              
-*------------------------------------------------------------*/
uint16_t FSM_Car_Moving_Proc(const uint8_t ctrl_elevator_ch_id)
{
	elevator_ctrl_and_status_t *cur_elevator_ptr;
	uint16_t ret_status;
	uint8_t limit_sw_cur_floor, limit_sw_min_floor, limit_sw_max_floor, cur_floor, car_movement_state;
  
	 if(ctrl_elevator_ch_id >= MAX_NUM_ELEVATORS)
	{
		appl_status_flag = ERR_ELEVATOR_CH_ID_EXCEEDS;
	    Error_or_Warning_Proc("14.08.01", ERROR_OCCURED, appl_status_flag);
	    return appl_status_flag;
    }	
    cur_elevator_ptr = elevator_ctrl_and_status + ctrl_elevator_ch_id;
    printf("\n Enter cur_floor : ");
	scanf("%u", &cur_floor);
	if((ret_status = Car_Movement_Direction(ctrl_elevator_ch_id, cur_floor, &car_movement_state)) != SUCCESS)
	{
		 appl_status_flag = ERR_CAR_MOVE_STATE_PROC;
	     Error_or_Warning_Proc("14.08.11", ERROR_OCCURED, appl_status_flag);
	     printf("ERR: event - car movement proc invalid \n");	        
		 return FAILURE;   	 
	}
	switch(car_movement_state)
	{
			   case MOVE_UP:			     
			   case MOVED_UP_STATIONARY:
                 if(cur_elevator_ptr->cur_floor + 1 == cur_floor)
				 {
					 cur_elevator_ptr->cur_floor = cur_floor;
                     switch(car_movement_state)
					 {
                         case MOVED_UP_STATIONARY:
						   #ifdef TRACE_FLOW
	                           printf("MOVING -> USER_ENTRY_AND_EXIT \n");	                  
	                       #endif
						    cur_elevator_ptr->elevator_status = MOVED_UP_STATIONARY;
		                   cur_elevator_ptr->cur_fsm_state =  FSM_USER_ENTRY_AND_EXIT;			               					  
						 break;
						 case MOVE_UP:						   
						    #ifdef TRACE_INFO
							   uint32_temp_disp_data = cur_elevator_ptr->cur_floor;
	                           printf("TRA: Moving Up non stop - cur_floor: %u, ", uint32_temp_disp_data); 
							   uint32_temp_disp_data = cur_elevator_ptr->next_stop_floor;
	                           printf("stop floor %u \n", uint32_temp_disp_data);           
	                        #endif
						 break;
					 }						 
			     }
				 else
				 {
					 #ifdef TRACE_ERROR
	                     printf("MOVING -> ABNORMAL_EVENT \n");	                    
	                 #endif					 
					 if(cur_elevator_ptr->cur_floor == cur_floor)
					 {
						#ifdef TRACE_ERROR
						   printf("ERR: event - move up but stationary \n");  
                        #endif  
                        cur_elevator_ptr->elevator_status = ERR_MOVE_UP_BUT_UNMOVED; 
						appl_status_flag = ERR_CAR_UNMOVED;
					    Error_or_Warning_Proc("14.08.02", ERROR_OCCURED, appl_status_flag);                         						
					 }
					 else
					 {
						 #ifdef TRACE_ERROR
						    printf("ERR: Event - move up next floor invalid \n");
						 #endif 
						 cur_elevator_ptr->elevator_status = ERR_MOVE_UP_NEXT_FLOOR_INVALID;
						 appl_status_flag = ERR_NEXT_FLOOR_INVALID;
					    Error_or_Warning_Proc("14.08.03", ERROR_OCCURED, appl_status_flag);  	
					 }					 
				     cur_elevator_ptr->cur_fsm_state =  FSM_ABNORMAL_EVENT; 
					 return FAILURE;
				 }
               break;
               case MOVE_DOWN:			     
			   case MOVED_DOWN_STATIONARY:
                 if(cur_elevator_ptr->cur_floor - 1 == cur_floor)
				 {
					  cur_elevator_ptr->cur_floor = cur_floor;
					  switch(car_movement_state)
					  {
                         case MOVED_DOWN_STATIONARY:
						   #ifdef TRACE_FLOW
	                           printf("MOVING -> USER_ENTRY_AND_EXIT \n");	                  
	                       #endif
						    cur_elevator_ptr->elevator_status = MOVED_DOWN_STATIONARY;
		                   cur_elevator_ptr->cur_fsm_state = FSM_USER_ENTRY_AND_EXIT;			               					   
						 break;
						 case MOVE_DOWN:			    
                            					 
						    #ifdef TRACE_INFO
							   uint32_temp_disp_data = cur_elevator_ptr->cur_floor;
	                           printf("TRA: Moving Down non stop - cur_floor: %u, ", uint32_temp_disp_data); 
							   uint32_temp_disp_data = cur_elevator_ptr->next_stop_floor;
	                           printf("stop floor %u \n", uint32_temp_disp_data); 
	                       #endif
						 break;
					  }		
				 }
				 else
				 {					 
					 #ifdef TRACE_ERROR
	                     printf("MOVING -> ABNORMAL_EVENT \n");	                    
	                 #endif					 
					 if(cur_elevator_ptr->cur_floor == cur_floor)
					 {
						#ifdef TRACE_ERROR
						   printf("ERR: event - move down but stationary\n");  
                        #endif  
                        cur_elevator_ptr->elevator_status = ERR_MOVE_DOWN_BUT_UNMOVED; 
                        appl_status_flag = ERR_CAR_UNMOVED;
					    Error_or_Warning_Proc("14.08.04", ERROR_OCCURED, appl_status_flag);  	  						
					 }
					 else
					 {
						 #ifdef TRACE_ERROR
						    printf("ERR: Event - move down next floor invalid \n");
						 #endif 
						 cur_elevator_ptr->elevator_status = ERR_MOVE_DOWN_NEXT_FLOOR_INVALID;
						 appl_status_flag = ERR_NEXT_FLOOR_INVALID;
					     Error_or_Warning_Proc("14.08.05", ERROR_OCCURED, appl_status_flag);  
					 }
				      cur_elevator_ptr->cur_fsm_state =  FSM_ABNORMAL_EVENT;
                      return FAILURE;					  
				 }	
               break;
			   default:
                  appl_status_flag = ERR_FORMAT_INVALID;
	              Error_or_Warning_Proc("14.08.14", ERROR_OCCURED, appl_status_flag);
	              return FAILURE;	
		}
       		
	return SUCCESS;
}
 
 /*------------------------------------------------------------*
FUNCTION NAME  :  FSM_User_Entry_And_Exit_Proc

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 14.17

BUGS           :              
-*------------------------------------------------------------*/
uint16_t FSM_User_Entry_And_Exit_Proc(const uint8_t ctrl_elevator_ch_id)
{
	elevator_ctrl_and_status_t *cur_elevator_ptr;
	uint16_t ret_status;
	uint8_t elevator_status, cur_floor_state, stop_floor_call_in_car_and_move;
	static uint8_t loop_flag = 0;
	
	 if(ctrl_elevator_ch_id >= MAX_NUM_ELEVATORS)
	{
		appl_status_flag = ERR_ELEVATOR_CH_ID_EXCEEDS;
	    Error_or_Warning_Proc("14.17.01", ERROR_OCCURED, appl_status_flag);
	    return appl_status_flag;
    }	
    cur_elevator_ptr = elevator_ctrl_and_status + ctrl_elevator_ch_id;
	if(((loop_flag & (1 << 3)) == 0))
	{
	    if(cur_elevator_ptr->before_fsm_state == FSM_PREPARE_USER_ENTRY_AND_EXIT  )
	    {			
	       if(cur_elevator_ptr->next_stop_floor != cur_elevator_ptr->cur_floor)
	       {
	  	       appl_status_flag = ERR_STOP_NOT_CUR_FLOOR;
	           Error_or_Warning_Proc("14.17.02", ERROR_OCCURED, appl_status_flag);
		       #ifdef TRACE_ERROR
		    	  printf("USER_ENTRY_AND_EXIT -> ABNORMAL_EVENT \n"); 
		          printf("ERR: event - next stop floor != cur_floor\n");
		       #endif
		       cur_elevator_ptr->elevator_status = ERR_NEXT_STOP_FLOOR_NOT_STOPPED;
		       cur_elevator_ptr->cur_fsm_state = FSM_ABNORMAL_EVENT;
               return SUCCESS;	
	        }
			if((cur_elevator_ptr->pending_floor_call_bit_field & (1 << cur_elevator_ptr->cur_floor)) == 0)
			{
		        appl_status_flag = ERR_NEXT_STOP_FLOOR_PROC;
		        Error_or_Warning_Proc("14.17.03", ERROR_OCCURED, appl_status_flag);  
		        #ifdef TRACE_ERROR
		        	printf("USER_ENTRY_AND_EXIT -> ABNORMAL_EVENT \n"); 
			        printf("ERR: event - next_stop but not stop list\n");
			    #endif
			    cur_elevator_ptr->elevator_status = ERR_NEXT_STOP_FLOOR_REACH_BUT_NOT_IN_STOP_LIST;
			    cur_elevator_ptr->cur_fsm_state = FSM_ABNORMAL_EVENT;
                return FAILURE;	
			}
			
	    	if((cur_elevator_ptr->in_car_floor_call_bit_field & (1 << cur_elevator_ptr->cur_floor)))
	        {
				 cur_elevator_ptr->in_car_floor_call_bit_field &= ~(1 << cur_elevator_ptr->cur_floor);
			     --cur_elevator_ptr->num_pending_calls_in_car;
			     if(cur_elevator_ptr->num_pending_calls_in_car == 0 && cur_elevator_ptr->in_car_floor_call_bit_field == 0)
			     {
				     cur_elevator_ptr->cur_min_floor_call_in_car = max_num_floors + 1;
	                 cur_elevator_ptr->cur_max_floor_call_in_car = max_num_floors + 1;
			     }
	         }
	         switch(cur_elevator_ptr->elevator_status)
	         {
		          case MOVED_UP_STATIONARY:
		             if((cur_elevator_ptr->hall_floor_call_up_bit_field & (1 << cur_elevator_ptr->cur_floor)))
	                 {
						  cur_elevator_ptr->hall_floor_call_up_bit_field &= ~(1 << cur_elevator_ptr->cur_floor);
				         --cur_elevator_ptr->num_pending_calls_up;
				         if(cur_elevator_ptr->num_pending_calls_up == 0 && cur_elevator_ptr->hall_floor_call_up_bit_field == 0)
			             {
				            cur_elevator_ptr->cur_min_floor_call_up = max_num_floors + 1;
	                        cur_elevator_ptr->cur_max_floor_call_up = max_num_floors + 1;
			             }
	                 }
					 if(cur_elevator_ptr->num_pending_calls_down != 0 && cur_elevator_ptr->cur_floor == cur_elevator_ptr->cur_max_floor_call_down && ((cur_elevator_ptr->hall_floor_call_down_bit_field & (1 << cur_elevator_ptr->cur_floor))))
	                 {
						cur_elevator_ptr->hall_floor_call_down_bit_field &= ~(1 << cur_elevator_ptr->cur_floor);
			        	--cur_elevator_ptr->num_pending_calls_down;
			        	if(cur_elevator_ptr->num_pending_calls_down == 0 && cur_elevator_ptr->hall_floor_call_down_bit_field == 0)
			            {
				             cur_elevator_ptr->cur_min_floor_call_down = max_num_floors + 1;
	                         cur_elevator_ptr->cur_max_floor_call_down = max_num_floors + 1;
			            }
			         }
		          break;
		          case MOVED_DOWN_STATIONARY:
		              if((cur_elevator_ptr->hall_floor_call_down_bit_field & (1 << cur_elevator_ptr->cur_floor)))
	                  {
		                   cur_elevator_ptr->hall_floor_call_down_bit_field &= ~(1 << cur_elevator_ptr->cur_floor);
			        	   --cur_elevator_ptr->num_pending_calls_down;
			        	   if(cur_elevator_ptr->num_pending_calls_down == 0 && cur_elevator_ptr->hall_floor_call_down_bit_field == 0)
			               {
				               cur_elevator_ptr->cur_min_floor_call_down = max_num_floors + 1;
	                           cur_elevator_ptr->cur_max_floor_call_down = max_num_floors + 1;
			               }
			          }
					 if(cur_elevator_ptr->num_pending_calls_up != 0 && cur_elevator_ptr->cur_floor == cur_elevator_ptr->cur_min_floor_call_up && ((cur_elevator_ptr->hall_floor_call_up_bit_field & (1 << cur_elevator_ptr->cur_floor))))
	                 {
						cur_elevator_ptr->hall_floor_call_up_bit_field &= ~(1 << cur_elevator_ptr->cur_floor);
			        	--cur_elevator_ptr->num_pending_calls_up;
			        	if(cur_elevator_ptr->num_pending_calls_up == 0 && cur_elevator_ptr->hall_floor_call_up_bit_field == 0)
			            {
				             cur_elevator_ptr->cur_min_floor_call_up = max_num_floors + 1;
	                         cur_elevator_ptr->cur_max_floor_call_up = max_num_floors + 1;
			            }
			         }
		          break;
	         }	 
	         cur_floor_state = (cur_elevator_ptr->in_car_floor_call_bit_field & (1 << cur_elevator_ptr->cur_floor)) || (cur_elevator_ptr->hall_floor_call_up_bit_field & (1 << cur_elevator_ptr->cur_floor))|| (cur_elevator_ptr->hall_floor_call_down_bit_field & (1 << cur_elevator_ptr->cur_floor)) ;
	         if(cur_floor_state == 0)
	         {
	             cur_elevator_ptr->pending_floor_call_bit_field &= ~(1 << cur_elevator_ptr->cur_floor);
		         --cur_elevator_ptr->num_pending_calls;
		        if(cur_elevator_ptr->num_pending_calls == 0 && cur_elevator_ptr->pending_floor_call_bit_field == 0)
		        {
				    cur_elevator_ptr->cur_min_floor_call = max_num_floors + 1;
	                cur_elevator_ptr->cur_max_floor_call = max_num_floors + 1;
		        }
	         }
	         #ifdef TRACE_INFO	    
		         printf("TRA: serviced floor call: %u\n", cur_elevator_ptr->cur_floor);				           
	         #endif 
	    }
	    loop_flag |= (1 << 3); 
	}
	cur_elevator_ptr->before_fsm_state = FSM_PREPARE_USER_ENTRY_AND_EXIT;
    if((ret_status = Compute_Floor_Stop_Datas(ctrl_elevator_ch_id, DOOR_OPENED_STAGE, &elevator_status)) != SUCCESS)
	{
	     appl_status_flag = ERR_NEXT_STOP_FLOOR_PROC;
    	 Error_or_Warning_Proc("14.23.02", ERROR_OCCURED, appl_status_flag);   
		 cur_elevator_ptr->elevator_status = ERR_COMPUTE_NEXT_STOP_FLOOR;
		 cur_elevator_ptr->cur_fsm_state = FSM_ABNORMAL_EVENT;
		 return SUCCESS;
	}	
    switch(elevator_status)
	{
        case NO_PENDING_FLOOR_CALLS:
		case STARTUP_STATIONARY:
          return SUCCESS;      		  
	   // break;
        default:  
           loop_flag &= ~(1 << 3);  		
           #ifdef TRACE_FLOW
	        printf("USER_ENTRY_AND_EXIT -> COMPUTE_NEXT_STOP_FLOOR\n"); 
          #endif
          cur_elevator_ptr->cur_fsm_state = FSM_COMPUTE_NEXT_STOP_FLOOR;
	} 
	return SUCCESS;
}

/*------------------------------------------------------------*
FUNCTION NAME  : FSM_Compute_Next_Stop_Floor_Proc 

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 14.23

BUGS           :              
-*------------------------------------------------------------*/
uint16_t FSM_Compute_Next_Stop_Floor_Proc(const uint8_t ctrl_elevator_ch_id)
{
	elevator_ctrl_and_status_t *cur_elevator_ptr;
	uint16_t ret_status;
	uint8_t cur_floor, elevator_status = 0;

	
	 if(ctrl_elevator_ch_id >= MAX_NUM_ELEVATORS)
	{
		appl_status_flag = ERR_ELEVATOR_CH_ID_EXCEEDS;
	    Error_or_Warning_Proc("14.23.01", ERROR_OCCURED, appl_status_flag);
	    return appl_status_flag;
    }	
    cur_elevator_ptr = elevator_ctrl_and_status + ctrl_elevator_ch_id;
	
	if((ret_status = Compute_Floor_Stop_Datas(ctrl_elevator_ch_id, DOOR_CLOSED_STAGE, &elevator_status)) != SUCCESS)
	{
	     appl_status_flag = ERR_NEXT_STOP_FLOOR_PROC;
    	 Error_or_Warning_Proc("14.23.02", ERROR_OCCURED, appl_status_flag);   
		 cur_elevator_ptr->elevator_status = ERR_COMPUTE_NEXT_STOP_FLOOR;
		 cur_elevator_ptr->cur_fsm_state = FSM_ABNORMAL_EVENT;
		 return SUCCESS;
	}
	switch(elevator_status)
	{
		case NO_PENDING_FLOOR_CALLS:
		   #ifdef TRACE_ERROR
	          printf("COMPUTE_NEXT_STOP_FLOOR -> ABNORMAL_EVENT \n");
	          printf("ERR: Event - no pending floor call\n");
	       #endif
		   cur_elevator_ptr->elevator_status = ERR_COMPUTE_NEXT_STOP_FLOOR;
	       cur_elevator_ptr->cur_fsm_state = FSM_ABNORMAL_EVENT;
           return SUCCESS;		   
		//break;
		case ONE_PENDING_FLOOR_CALL:
		  //one pending floor calls
        case STARTUP_STATIONARY:
           //startup stationary		
        case TRIGGER_MOVE_UP_NO_DIR_CHANGE:						
		    //trigger car to move up		
		case TRIGGER_MOVE_UP_DIR_CHANGE:
		  //trigger car to move up
		case TRIGGER_MOVE_DOWN_NO_DIR_CHANGE:						
		   //trigger car to move down		  
		case TRIGGER_MOVE_DOWN_DIR_CHANGE:
		   //trigger car to move down
		   #ifdef TRACE_DATA
		       printf("After compute floor stop datas - Closed Stage \n");
               printf("================================================\n");
                Disp_Oper_Proc(CTRL_ELEVATOR_CH_ID);
               printf("================================================\n");
               #elseif defined TRACE_INFO
                 printf("After compute floor stop datas - Closed Stage \n");
                 printf("================================================\n");
                 printf("cur_floor                     : %u\n", cur_elevator_ptr->cur_floor);
	             printf("next_stop_floor               : %u\n", cur_elevator_ptr->next_stop_floor);
			     printf("pending floor calls           : 0x%X\n", cur_elevator_ptr->pending_floor_call_bit_field);
			     printf("================================================\n");
			    #endif
               #ifdef TRACE_FLOW
	           printf("COMPUTE_NEXT_STOP_FLOOR -> DECIDE_CAR_MOVEMENT \n");  
		   #endif
          	cur_elevator_ptr->cur_fsm_state = FSM_DECIDE_CAR_MOVEMENT;	   
		break;  
        default:	
            appl_status_flag = ERR_FORMAT_INVALID;
		    Error_or_Warning_Proc("14.23.03", ERROR_OCCURED, appl_status_flag);
		   	return FAILURE;			   
	}

	return SUCCESS;
}

/*------------------------------------------------------------*
FUNCTION NAME  : Compute_Floor_Stop_Datas

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 11.22

BUGS           :              
-*------------------------------------------------------------*/
uint16_t Compute_Floor_Stop_Datas(const uint8_t ctrl_elevator_ch_id, const uint8_t stage_type, uint8_t *const elevator_status_ptr)
{
	elevator_ctrl_and_status_t *cur_elevator_ptr;
	uint16_t ret_status;
	uint8_t stop_floor_call_in_car_and_move, stop_floor_call_move;
	static uint8_t proc_bit_field = (1 << 0) | (1 << 1);
	static int8_t cur_floor;
	
	if(elevator_status_ptr == NULL_PTR)
	{
		appl_status_flag = ERR_NULL_PTR;
	    Error_or_Warning_Proc("11.22.01", ERROR_OCCURED, appl_status_flag);
	    return appl_status_flag;
	}
	switch(stage_type)
	{
		case DOOR_OPENED_STAGE:
	    case DOOR_CLOSED_STAGE:
		break;
		default:
		   	appl_status_flag = ERR_FORMAT_INVALID;
	        Error_or_Warning_Proc("11.22.01", ERROR_OCCURED, appl_status_flag);
	        return appl_status_flag;  		
	}
	if(ctrl_elevator_ch_id >= MAX_NUM_ELEVATORS)
	{
		appl_status_flag = ERR_ELEVATOR_CH_ID_EXCEEDS;
	    Error_or_Warning_Proc("11.22.02", ERROR_OCCURED, appl_status_flag);
	    return appl_status_flag;
    }	
    cur_elevator_ptr = elevator_ctrl_and_status + ctrl_elevator_ch_id;
	if(cur_elevator_ptr->pending_floor_call_bit_field == 0)
	{
		//no more floor calls 
		 Reset_Elevator_Datas(ctrl_elevator_ch_id, RESET_STOP_DATAS);
		 #ifdef TRACE_INFO
		     printf("INFO: No Pending Floor Calls \n");
		 #endif	 
		 *elevator_status_ptr = NO_PENDING_FLOOR_CALLS;
		 return SUCCESS;
	}
	switch(cur_elevator_ptr->elevator_status)
    { 
      	case MOVED_UP_STATIONARY:
		    if(cur_elevator_ptr->num_pending_calls_in_car != 0) 
 			{
			   if(cur_elevator_ptr->num_pending_calls_up != 0)
			   {
				   stop_floor_call_in_car_and_move = (cur_elevator_ptr->cur_max_floor_call_in_car > cur_elevator_ptr->cur_max_floor_call_up) ? cur_elevator_ptr->cur_max_floor_call_in_car : cur_elevator_ptr->cur_max_floor_call_up;
			   }
			   else
			   {
				   stop_floor_call_in_car_and_move = cur_elevator_ptr->cur_max_floor_call_in_car;
			   }
			}
            else 
			{
                if(cur_elevator_ptr->num_pending_calls_up != 0)
			    {
				     stop_floor_call_in_car_and_move = cur_elevator_ptr->cur_max_floor_call_up;
				}
				else
				{
					 stop_floor_call_in_car_and_move = cur_elevator_ptr->cur_floor;
				}
			}
            if(cur_elevator_ptr->num_pending_calls_down != 0)
			{
				 if((cur_elevator_ptr->cur_max_floor_call_down >= stop_floor_call_in_car_and_move ))
				 {
					 stop_floor_call_move = cur_elevator_ptr->cur_max_floor_call_down;
				 }
                 else
				 {
					  stop_floor_call_move = stop_floor_call_in_car_and_move;
				 }					 
			}
            else
			{
				 stop_floor_call_move = stop_floor_call_in_car_and_move;
			}
			if(stop_floor_call_move != cur_elevator_ptr->cur_floor)
			{ 
		        cur_elevator_ptr->cur_max_floor_call = stop_floor_call_move;
				#ifdef TRACE_DATA
			       printf("TRA_01: MOVE_UP, max_call: %u \n", cur_elevator_ptr->cur_max_floor_call);
			    #endif
			}
			#ifdef TRACE_DATA
			   printf("TRA_02: MOVE_UP, in_car_and_move: %u & move : %u \n", stop_floor_call_in_car_and_move, stop_floor_call_move);
			#endif 			
		    if(stop_floor_call_in_car_and_move != cur_elevator_ptr->cur_floor && cur_elevator_ptr->cur_floor <= stop_floor_call_in_car_and_move)
			{
				#ifdef TRACE_DATA
			      printf("TRA_03: MOVE_UP, cur_floor: %u & in_car_move : %u \n", cur_elevator_ptr->cur_floor + 1, stop_floor_call_in_car_and_move);
			    #endif 
				for(cur_floor = cur_elevator_ptr->cur_floor + 1; cur_floor <= stop_floor_call_in_car_and_move; ++cur_floor)
			    {
				    if((cur_elevator_ptr->in_car_floor_call_bit_field & (1 << cur_floor)) || (cur_elevator_ptr->hall_floor_call_up_bit_field & (1 << cur_floor)))
			        {
				        switch(stage_type)
				        {
			             	case DOOR_OPENED_STAGE:					   
			    	           if((cur_elevator_ptr->in_car_floor_call_bit_field & (1 << cur_floor))) 
			    	           {
								    #ifdef TRACE_DATA
			                             printf("TRA_04: HALL_UP & IN_CAR: MOVE_UP, call_in_car : %u \n", cur_floor);
			                        #endif
								   if(cur_elevator_ptr->cur_min_floor_call_in_car == cur_elevator_ptr->cur_floor)
								   {
								      cur_elevator_ptr->cur_min_floor_call_in_car = cur_floor;
									  #ifdef TRACE_DATA
			                             printf("TRA_05: HALL_UP & IN_CAR: MOVE_UP, min_in_car : %u \n", cur_elevator_ptr->cur_min_floor_call_in_car);
			                          #endif
								   }
						       }
							   if((cur_elevator_ptr->hall_floor_call_up_bit_field & (1 << cur_floor))) 
							   {
								   #ifdef TRACE_DATA
			                             printf("TRA_06: HALL_UP & IN_CAR: MOVE_UP, call_up : %u \n", cur_floor);
			                        #endif
								   if(cur_elevator_ptr->cur_min_floor_call_up == cur_elevator_ptr->cur_floor)
								   {
								       cur_elevator_ptr->cur_min_floor_call_up = cur_floor;
                                        #ifdef TRACE_DATA
			                               printf("TRA_07: HALL_UP & IN_CAR: MOVE_UP, min_up : %u \n", cur_floor);
			                            #endif									   
								   }
							   }			   
					        break;
					        case DOOR_CLOSED_STAGE:					   	
					          cur_elevator_ptr->next_stop_floor = cur_floor;
					          #ifdef TRACE_DATA
			                     printf("TRA_08: HALL_UP & IN_CAR: MOVE_UP, stop_floor : %u \n", cur_floor);
			                 #endif	
					        break;
						}
						break;
					}		
			   	 }
			     if(cur_floor > cur_elevator_ptr->cur_max_floor_call)
			     {
					 #ifdef TRACE_ERROR
					    printf("COMPUTE_NEXT_STOP_FLOOR -> ABNORMAL_EVENT \n");
					    uint32_temp_disp_data = cur_elevator_ptr->cur_max_floor_call;						
			            printf("ERR: event - max_call: %u but not in stop list \n", uint32_temp_disp_data);
				     #endif		
					 appl_status_flag = ERR_MAX_STOP_FLOOR_REACH_BUT_NOT_IN_STOP_LIST;
	                 Error_or_Warning_Proc("11.22.03", ERROR_OCCURED, appl_status_flag);
	                 return appl_status_flag;
			     }
				 #ifdef TRACE_DATA				     
					  printf("TRA_09: HALL_UP & IN_CAR: MOVE_UP, TRIGGER_MOVE_UP_NO_DIR_CHANGE \n"); 
				 #endif				  
				 *elevator_status_ptr = TRIGGER_MOVE_UP_NO_DIR_CHANGE;
				 return SUCCESS;
			 }
		     if(cur_elevator_ptr->num_pending_calls_down != 0 && cur_elevator_ptr->cur_max_floor_call_down > stop_floor_call_in_car_and_move)
		     {
				if((proc_bit_field & (1 << 0)) )
				{
					 #ifdef TRACE_DATA				     
					    printf("TRA_10: HALL_DOWN: MOVE_UP, max_down: %u > in_car_and_move: %u\n",cur_elevator_ptr->cur_max_floor_call_down, stop_floor_call_in_car_and_move ); 
				     #endif	
					switch(stage_type)
					{
						case DOOR_OPENED_STAGE:   
					    break;
						case DOOR_CLOSED_STAGE:
						   cur_elevator_ptr->next_stop_floor = cur_elevator_ptr->cur_max_floor_call_down;
					       #ifdef TRACE_DATA				     
					           printf("TRA_13: HALL_DOWN: MOVE_UP, next_stop: %u \n", cur_elevator_ptr->next_stop_floor); 
				           #endif	
				           proc_bit_field &= ~(1 << 0);  
						break;   
					}					
				    #ifdef TRACE_FLOW			     
					    printf("TRA_11: HALL_DOWN: MOVE_UP, TRIGGER_MOVE_UP_NO_DIR_CHANGE \n"); 
				    #endif				  
				    *elevator_status_ptr = TRIGGER_MOVE_UP_NO_DIR_CHANGE;
				   return SUCCESS; 
				}
			 }
			//action to move down
			//cur_elevator_ptr->cur_max_floor_call_down == stop_floor_call_in_car_and_move
			 if(cur_elevator_ptr->num_pending_calls_down != 0 )
		     {
			        if(cur_elevator_ptr->num_pending_calls_in_car != 0)
			        {
					    stop_floor_call_in_car_and_move = (cur_elevator_ptr->cur_min_floor_call_in_car < cur_elevator_ptr->cur_min_floor_call_down) ? cur_elevator_ptr->cur_min_floor_call_in_car : cur_elevator_ptr->cur_min_floor_call_down;
			        }
			        else
			        {
				       stop_floor_call_in_car_and_move = cur_elevator_ptr->cur_min_floor_call_down;
				    }
			 }
		     else
		     {
				   if(cur_elevator_ptr->num_pending_calls_in_car != 0)
			       {
				        stop_floor_call_in_car_and_move = cur_elevator_ptr->cur_min_floor_call_in_car;
				   }
				   else
			       {
				     	 stop_floor_call_in_car_and_move = cur_elevator_ptr->cur_floor;
			       }      
		    }
		    if(cur_elevator_ptr->num_pending_calls_up != 0)
		    {
				   if((cur_elevator_ptr->cur_min_floor_call_up < stop_floor_call_in_car_and_move ))
				   {
				  	   stop_floor_call_move = cur_elevator_ptr->cur_min_floor_call_up;
				   }
                   else
				   {
				   	  stop_floor_call_move = stop_floor_call_in_car_and_move;
				   }					 
			}
		    else
		    {
			       	 stop_floor_call_move = stop_floor_call_in_car_and_move;
			}
			#ifdef TRACE_DATA
			     printf("TRA_12: HALL_DOWN & IN_CAR: MOVE_UP, in_car_and_move: %u & move : %u \n", stop_floor_call_in_car_and_move, stop_floor_call_move);
		    #endif 
			if(stop_floor_call_move != cur_elevator_ptr->cur_floor)
			{
		         cur_elevator_ptr->cur_min_floor_call = stop_floor_call_move;
				 #ifdef TRACE_DATA
			        printf("TRA_13: HALL_DOWN & IN_CAR: MOVE_UP, min_call: %u \n", cur_elevator_ptr->cur_min_floor_call);
		        #endif 
			}
			#ifdef TRACE_DATA
			    printf("TRA_14: HALL_DOWN & IN_CAR: MOVE_UP, cur_floor: %u, in_car_and_move: %u \n", cur_elevator_ptr->cur_floor, stop_floor_call_in_car_and_move);
		    #endif
			if(cur_elevator_ptr->num_pending_calls_up != 0)
			{
				 switch(stage_type)
				 {
					case DOOR_OPENED_STAGE:	
					   for(cur_floor = cur_elevator_ptr->cur_max_floor_call_up - 1; cur_floor >= cur_elevator_ptr->cur_min_floor_call_up; --cur_floor) 
					   {
						 	 if((cur_elevator_ptr->hall_floor_call_up_bit_field & (1 << cur_floor))) 
						     {
							    cur_elevator_ptr->cur_max_floor_call_up = cur_floor;									
								#ifdef TRACE_DATA
			                         printf("TRA_15: HALL_UP: MOVE_UP, max_up : %u \n", cur_elevator_ptr->cur_max_floor_call_up);
			                    #endif
						    	break;
							 }	
					   }      
					break;  
					case DOOR_CLOSED_STAGE:	                       					
					break; 
				 }		
			 }
             if(stop_floor_call_in_car_and_move != cur_elevator_ptr->cur_floor && cur_elevator_ptr->cur_floor >= stop_floor_call_in_car_and_move)
		     {	
		         for(cur_floor = cur_elevator_ptr->cur_floor - 1; cur_floor >= (int8_t)stop_floor_call_in_car_and_move; --cur_floor)
				 {
			 	    if((cur_elevator_ptr->in_car_floor_call_bit_field & (1 << cur_floor)) || (cur_elevator_ptr->hall_floor_call_down_bit_field & (1 << cur_floor)))
			   	    {
			           switch(stage_type)						
			     	   {
			    	     	case DOOR_OPENED_STAGE:						  
					      
						           if((cur_elevator_ptr->hall_floor_call_down_bit_field & (1 << cur_floor)))
						           {
								      cur_elevator_ptr->cur_max_floor_call_down = cur_floor; 
									  #ifdef TRACE_DATA
			                             printf("TRA_16: HALL_DOWN & IN_CAR: MOVE_UP, max_down : %u \n", cur_elevator_ptr->cur_max_floor_call_down );
		                              #endif
							       }
							       if((cur_elevator_ptr->in_car_floor_call_bit_field & (1 << cur_floor)))
							       {
								      cur_elevator_ptr->cur_max_floor_call_in_car = cur_floor; 
									  #ifdef TRACE_DATA
			                             printf("TRA_17: HALL_DOWN & IN_CAR: MOVE_UP, max_in_car : %u \n", cur_elevator_ptr->cur_max_floor_call_in_car );
		                              #endif
							       }
			   	        	       cur_elevator_ptr->cur_max_floor_call = cur_floor;
                                   #ifdef TRACE_DATA
			                           printf("TRA_18: HALL_DOWN & IN_CAR: MOVE_UP, max_call : %u \n", cur_elevator_ptr->cur_max_floor_call );
		                           #endif 
								    					
						    break;
						    case DOOR_CLOSED_STAGE:		
			                   cur_elevator_ptr->next_stop_floor = cur_floor;
						       #ifdef TRACE_DATA
			                     printf("TRA_19: HALL_DOWN & IN_CAR: MOVE_UP, next_stop: %u \n", cur_elevator_ptr->next_stop_floor );
		                       #endif  
					        break;
					    }
						break;
 					}
				 }			
					 if(cur_floor < cur_elevator_ptr->cur_min_floor_call)
			         {
				        #ifdef TRACE_ERROR
			              printf("COMPUTE_NEXT_STOP_FLOOR -> ABNORMAL_EVENT \n");
					      printf("ERR: event - move down cur : %u  < min_floor_call : %u \n",cur_floor, cur_elevator_ptr->cur_min_floor_call );
				        #endif
				        appl_status_flag = ERR_MIN_STOP_FLOOR_INVALID;
	                    Error_or_Warning_Proc("11.22.05", ERROR_OCCURED, appl_status_flag);
	                    return appl_status_flag; 
			         }
			         #ifdef TRACE_FLOW
			             printf("TRA_20: HALL_DOWN & IN_CAR: MOVE_UP, TRIGGER_MOVE_DOWN_DIR_CHANGE \n"); 
			         #endif
			         proc_bit_field |= (1 << 0);
			         *elevator_status_ptr = TRIGGER_MOVE_DOWN_DIR_CHANGE;
			         return SUCCESS;	
			  }
			 if(stop_floor_call_move != cur_elevator_ptr->cur_floor && cur_elevator_ptr->cur_min_floor_call_up < stop_floor_call_in_car_and_move)
			 {
				 switch(stage_type)
				 {
					case DOOR_OPENED_STAGE:	
					    if((cur_elevator_ptr->hall_floor_call_up_bit_field & (1 << cur_elevator_ptr->cur_min_floor_call_up)))
						{
							 cur_elevator_ptr->next_stop_floor = cur_elevator_ptr->cur_min_floor_call_up;
							 #ifdef TRACE_DATA
			                    printf("TRA_21: HALL_UP: MOVE_UP, TRIGGER_MOVE_DOWN_DIR_CHANGE, stop_floor : %u \n", cur_elevator_ptr->next_stop_floor); 
			                 #endif		
                             for(cur_floor = cur_elevator_ptr->cur_min_floor_call_up + 1; cur_floor <= cur_elevator_ptr->cur_max_floor_call_up; ++cur_floor) 
							 {
								 if((cur_elevator_ptr->hall_floor_call_up_bit_field & (1 << cur_floor))) 
							     {
								    cur_elevator_ptr->cur_min_floor_call_up = cur_floor;									
									#ifdef TRACE_DATA
			                             printf("TRA_22: HALL_UP: MOVE_UP, min_up : %u \n", cur_elevator_ptr->cur_min_floor_call_up);
			                        #endif
							    }	
							}								 
						}
					break;  
					case DOOR_CLOSED_STAGE:					  
					break; 
				}
				#ifdef TRACE_FLOW
			       printf("TRA_23: HALL_UP: MOVE_UP, UP TRIGGER_MOVE_DOWN_DIR_CHANGE \n"); 
			    #endif
			    proc_bit_field |= (1 << 0);
			    *elevator_status_ptr = TRIGGER_MOVE_DOWN_DIR_CHANGE;
			    return SUCCESS;				
			 }			  
		break;
		case MOVED_DOWN_STATIONARY:
		    if(cur_elevator_ptr->num_pending_calls_down != 0 )
		     {
			        if(cur_elevator_ptr->num_pending_calls_in_car != 0)
			        {
					    stop_floor_call_in_car_and_move = (cur_elevator_ptr->cur_min_floor_call_in_car < cur_elevator_ptr->cur_min_floor_call_down) ? cur_elevator_ptr->cur_min_floor_call_in_car : cur_elevator_ptr->cur_min_floor_call_down;
			        }
			        else
			        {
				       stop_floor_call_in_car_and_move = cur_elevator_ptr->cur_min_floor_call_down;
				    }
			 }
		     else
		     {
				   if(cur_elevator_ptr->num_pending_calls_in_car != 0)
			       {
				        stop_floor_call_in_car_and_move = cur_elevator_ptr->cur_min_floor_call_in_car;
				   }
				   else
			       {
				     	 stop_floor_call_in_car_and_move = cur_elevator_ptr->cur_floor;
			       }      
		    }
		    if(cur_elevator_ptr->num_pending_calls_up != 0)
		    {
				   if((cur_elevator_ptr->cur_min_floor_call_up < stop_floor_call_in_car_and_move ))
				   {
				  	   stop_floor_call_move = cur_elevator_ptr->cur_min_floor_call_up;
				   }
                   else
				   {
				   	  stop_floor_call_move = stop_floor_call_in_car_and_move;
				   }					 
			}
		    else
		    {
			       	 stop_floor_call_move = stop_floor_call_in_car_and_move;
			}
			#ifdef TRACE_DATA
			     printf("TRA_24: HALL_DOWN & IN_CAR: MOVE_DOWN, in_car_and_move: %u & move : %u \n", stop_floor_call_in_car_and_move, stop_floor_call_move);
		    #endif 
			if(stop_floor_call_move != cur_elevator_ptr->cur_floor)
			{
		         cur_elevator_ptr->cur_min_floor_call = stop_floor_call_move;
				 #ifdef TRACE_DATA
			        printf("TRA_25: HALL_DOWN & IN_CAR: MOVE_DOWN, min_call: %u \n", cur_elevator_ptr->cur_min_floor_call);
		        #endif 
			}
            if(stop_floor_call_in_car_and_move != cur_elevator_ptr->cur_floor && cur_elevator_ptr->cur_floor >= stop_floor_call_in_car_and_move)
		    {	                      		   
			         for(cur_floor = cur_elevator_ptr->cur_floor - 1; cur_floor >= (int8_t)stop_floor_call_in_car_and_move; --cur_floor)
					 {
			 	        if((cur_elevator_ptr->in_car_floor_call_bit_field & (1 << cur_floor)) || (cur_elevator_ptr->hall_floor_call_down_bit_field & (1 << cur_floor)))
			   	        {
			   	        	 switch(stage_type)						
			     	         {
			    	         	case DOOR_OPENED_STAGE:
						           if((cur_elevator_ptr->hall_floor_call_down_bit_field & (1 << cur_floor)))
						           {
								      cur_elevator_ptr->cur_max_floor_call_down = cur_floor; 
									  #ifdef TRACE_DATA
			                             printf("TRA_26: HALL_DOWN & IN_CAR: MOVE_DOWN, max_down : %u \n", cur_elevator_ptr->cur_max_floor_call_down );
		                              #endif
							       }
							       if((cur_elevator_ptr->in_car_floor_call_bit_field & (1 << cur_floor)))
							       {
								      cur_elevator_ptr->cur_max_floor_call_in_car = cur_floor; 
									  #ifdef TRACE_DATA
			                             printf("TRA_27: HALL_DOWN & IN_CAR: MOVE_DOWN, max_in_car : %u \n", cur_elevator_ptr->cur_max_floor_call_in_car );
		                              #endif
							       }
			   	        	       cur_elevator_ptr->cur_max_floor_call = cur_floor;
                                   #ifdef TRACE_DATA
			                           printf("TRA_28: HALL_DOWN & IN_CAR: MOVE_DOWN, max_call : %u \n", cur_elevator_ptr->cur_max_floor_call );
		                           #endif  								   
						        break;
						        case DOOR_CLOSED_STAGE:		
			            	       cur_elevator_ptr->next_stop_floor = cur_floor;
								   #ifdef TRACE_DATA
			                           printf("TRA_29: HALL_DOWN & IN_CAR: MOVE_DOWN, next_stop : %u \n", cur_elevator_ptr->next_stop_floor );
		                           #endif  
					            break;
					         }
			     	      break;
			            }
			         }
					 if(cur_floor < cur_elevator_ptr->cur_min_floor_call)
			         {
				        #ifdef TRACE_ERROR
			              printf("COMPUTE_NEXT_STOP_FLOOR -> ABNORMAL_EVENT \n");
					      printf("ERR: event - move down cur : %u  < min_call : %u \n",cur_floor, cur_elevator_ptr->cur_min_floor_call );
				        #endif
				        appl_status_flag = ERR_MIN_STOP_FLOOR_INVALID;
	                    Error_or_Warning_Proc("11.22.07", ERROR_OCCURED, appl_status_flag);
	                    return appl_status_flag; 
			         }
			         #ifdef TRACE_FLOW
			             printf("TRA_30: HALL_DOWN & IN_CAR: MOVE_DOWN, TRIGGER_MOVE_DOWN_NO_DIR_CHANGE \n"); 
			         #endif
			         *elevator_status_ptr = TRIGGER_MOVE_DOWN_NO_DIR_CHANGE;
			         return SUCCESS;	
			}
			if(cur_elevator_ptr->num_pending_calls_up != 0 && cur_elevator_ptr->cur_min_floor_call_up < stop_floor_call_in_car_and_move)
		    {
				if((proc_bit_field & (1 << 1)))
				{
					 #ifdef TRACE_DATA				     
					    printf("TRA_31: HALL_UP: MOVE_DOWN, min_up: %u < in_car_and_move : %u\n",cur_elevator_ptr->cur_min_floor_call_up, stop_floor_call_in_car_and_move ); 
				     #endif	
					switch(stage_type)
					{
					    case DOOR_OPENED_STAGE:
					    break;
						case DOOR_CLOSED_STAGE:	  
						     cur_elevator_ptr->next_stop_floor = cur_elevator_ptr->cur_min_floor_call_up;
					         #ifdef TRACE_DATA				     
					            printf("TRA_32: HALL-UP: MOVE_DOWN, next_stop: %u \n", cur_elevator_ptr->next_stop_floor); 
				             #endif	
							 proc_bit_field &= ~(1 << 1);                      				
						break;   
					}					
				    #ifdef TRACE_FLOW				     
					    printf("TRA_43: HALL_UP: MOVE_DOWN, TRIGGER_MOVE_DOWN_NO_DIR_CHANGE \n"); 
				    #endif				  
				    *elevator_status_ptr = TRIGGER_MOVE_DOWN_NO_DIR_CHANGE;
				   return SUCCESS; 
				}
			}
			//action to move up
			//cur_elevator_ptr->cur_min_floor_call_up == stop_floor_call_in_car_and_move
			if(cur_elevator_ptr->num_pending_calls_in_car != 0) 
 			{
			   if(cur_elevator_ptr->num_pending_calls_up != 0)
			   {
				   stop_floor_call_in_car_and_move = (cur_elevator_ptr->cur_max_floor_call_in_car > cur_elevator_ptr->cur_max_floor_call_up) ? cur_elevator_ptr->cur_max_floor_call_in_car : cur_elevator_ptr->cur_max_floor_call_up;
			   }
			   else
			   {
				   stop_floor_call_in_car_and_move = cur_elevator_ptr->cur_max_floor_call_in_car;
			   }
			}
            else 
			{
                if(cur_elevator_ptr->num_pending_calls_up != 0)
			    {
				     stop_floor_call_in_car_and_move = cur_elevator_ptr->cur_max_floor_call_up;
				}
				else
				{
					 stop_floor_call_in_car_and_move = cur_elevator_ptr->cur_floor;
				}
			}
            if(cur_elevator_ptr->num_pending_calls_down != 0)
			{
				 if((cur_elevator_ptr->cur_max_floor_call_down >= stop_floor_call_in_car_and_move ))
				 {
					 stop_floor_call_move = cur_elevator_ptr->cur_max_floor_call_down;
				 }
                 else
				 {
					  stop_floor_call_move = stop_floor_call_in_car_and_move;
				 }					 
			}
            else
			{
				 stop_floor_call_move = stop_floor_call_in_car_and_move;
			}
			#ifdef TRACE_DATA
			     printf("TRA_33: HALL_UP & IN_CAR: MOVE_DOWN, in_car_and_move: %u & move : %u \n", stop_floor_call_in_car_and_move, stop_floor_call_move);
		    #endif
			if(stop_floor_call_move != cur_elevator_ptr->cur_floor)
			{ 
		        cur_elevator_ptr->cur_max_floor_call = stop_floor_call_move;
				#ifdef TRACE_DATA
			       printf("TRA_34: HALL_UP & IN_CAR: MOVE_DOWN, max_call: %u \n", cur_elevator_ptr->cur_max_floor_call);
			    #endif
			}
			if(cur_elevator_ptr->num_pending_calls_down != 0)
			{
				 switch(stage_type)
				 {
					case DOOR_OPENED_STAGE:	
					   for(cur_floor = cur_elevator_ptr->cur_min_floor_call_down + 1; cur_floor <= cur_elevator_ptr->cur_max_floor_call_down; ++cur_floor) 
					   {
						 	 if((cur_elevator_ptr->hall_floor_call_down_bit_field & (1 << cur_floor))) 
						     {
							    cur_elevator_ptr->cur_min_floor_call_down = cur_floor;									
								#ifdef TRACE_DATA
			                         printf("TRA_35: HALL_DOWN: MOVE_DOWN, min_down : %u \n", cur_elevator_ptr->cur_min_floor_call_down);
			                    #endif
						    	break;
							 }	
					   }      
					break;  
					case DOOR_CLOSED_STAGE:	                       					
					break; 
				 }		
			 }
               if(stop_floor_call_in_car_and_move != cur_elevator_ptr->cur_floor && cur_elevator_ptr->cur_floor <= stop_floor_call_in_car_and_move)
		       {	                      		   
			         for(cur_floor = cur_elevator_ptr->cur_floor + 1; cur_floor <= stop_floor_call_in_car_and_move; ++cur_floor)
					 {
			 	        if((cur_elevator_ptr->in_car_floor_call_bit_field & (1 << cur_floor)) || (cur_elevator_ptr->hall_floor_call_up_bit_field & (1 << cur_floor)))
			   	        {
			   	        	 switch(stage_type)						
			     	         {
			    	         	case DOOR_OPENED_STAGE:
						           if((cur_elevator_ptr->hall_floor_call_up_bit_field & (1 << cur_floor)))
						           {
								      cur_elevator_ptr->cur_min_floor_call_up = cur_floor; 
									  #ifdef TRACE_DATA
			                             printf("TRA_36: HALL_UP & IN_CAR: MOVE_DOWN, min_up : %u \n", cur_elevator_ptr->cur_min_floor_call_up );
		                              #endif
							       }
							       if((cur_elevator_ptr->in_car_floor_call_bit_field & (1 << cur_floor)))
							       {
								      cur_elevator_ptr->cur_min_floor_call_in_car = cur_floor; 
									  #ifdef TRACE_DATA
			                             printf("TRA_36: HALL_UP & IN_CAR: MOVE_DOWN, min_in_car : %u \n", cur_elevator_ptr->cur_min_floor_call_in_car );
		                              #endif
							       }
			   	        	       cur_elevator_ptr->cur_min_floor_call = cur_floor;
                                   #ifdef TRACE_DATA
			                           printf("TRA_37: HALL_UP & IN_CAR: MOVE_DOWN , min : %u \n", cur_elevator_ptr->cur_min_floor_call );
		                           #endif  								   
						        break;
						        case DOOR_CLOSED_STAGE:		
			            	       cur_elevator_ptr->next_stop_floor = cur_floor;
								   #ifdef TRACE_DATA
			                           printf("TRA_38: HALL_UP & IN_CAR: MOVE_DOWN , stop_floor : %u \n", cur_elevator_ptr->next_stop_floor );
		                           #endif  
					            break;
					         }
			     	        break;
			            }
			         }
					 if(cur_floor > cur_elevator_ptr->cur_max_floor_call)
			         {
				        #ifdef TRACE_ERROR
			              printf("COMPUTE_NEXT_STOP_FLOOR -> ABNORMAL_EVENT \n");
					      printf("ERR: event - move up cur : %u  > max_call : %u \n",cur_floor, cur_elevator_ptr->cur_max_floor_call );
				        #endif
				        appl_status_flag = ERR_MIN_STOP_FLOOR_INVALID;
	                    Error_or_Warning_Proc("11.22.09", ERROR_OCCURED, appl_status_flag);
	                    return appl_status_flag; 
			         }
			         #ifdef TRACE_FLOW
			             printf("TRA_39: HALL_UP & IN_CAR MOVE_DOWN, TRIGGER_MOVE_UP_DIR_CHANGE \n"); 
			         #endif
			           proc_bit_field |=  (1 << 1); 
			         *elevator_status_ptr = TRIGGER_MOVE_UP_DIR_CHANGE;
			         return SUCCESS;	
			  }
			  
			  if(stop_floor_call_move != cur_elevator_ptr->cur_floor && cur_elevator_ptr->cur_max_floor_call_down > stop_floor_call_in_car_and_move)
			  {
				 switch(stage_type)
				 {
					case DOOR_OPENED_STAGE:	
					    if((cur_elevator_ptr->hall_floor_call_down_bit_field & (1 << cur_elevator_ptr->cur_max_floor_call_down)))
						{
							 cur_elevator_ptr->next_stop_floor = cur_elevator_ptr->cur_max_floor_call_down;
							 #ifdef TRACE_DATA
			                    printf("TRA_40: HALL_UP: MOVE_UP, TRIGGER_MOVE_DOWN_DIR_CHANGE, next_stop : %u \n", cur_elevator_ptr->next_stop_floor); 
			                 #endif		
							 for(cur_floor = cur_elevator_ptr->cur_max_floor_call_down - 1; cur_floor >= (int8_t)cur_elevator_ptr->cur_min_floor_call_down; --cur_floor )
							 {
								  if((cur_elevator_ptr->hall_floor_call_down_bit_field & (1 << cur_floor)))
						          {
									  #ifdef TRACE_DATA
			                             printf("TRA_57: HALL_DOWN: MOVE_DOWN, max_down : %u \n", cur_elevator_ptr->cur_max_floor_call_down );
		                              #endif
									  cur_elevator_ptr->cur_max_floor_call_down = cur_floor;  
							      }
							 }
						}
					break;  
					case DOOR_CLOSED_STAGE:					  
					break; 
				}
                #ifdef TRACE_FLOW
			       printf("TRA_41: HALL_DOWN MOVE_DOWN, TRIGGER_MOVE_UP_DIR_CHANGE \n"); 
			    #endif
			      proc_bit_field |=  (1 << 1); 
			    *elevator_status_ptr = TRIGGER_MOVE_UP_DIR_CHANGE;
			    return SUCCESS;				
			 }
		break;
	    case STARTUP_STATIONARY:
		  for(cur_floor = cur_elevator_ptr->cur_floor + 1; cur_floor <= cur_elevator_ptr->cur_max_floor_call; ++cur_floor)
		  {
		  	   if((cur_elevator_ptr->pending_floor_call_bit_field & (1 << cur_floor)) )
			   { 
			     ++stop_floor_call_move;
			   }
		  }
		  for(cur_floor = cur_elevator_ptr->cur_floor - 1; cur_floor >= (int8_t)cur_elevator_ptr->cur_min_floor_call; --cur_floor)
		  {
		  	   if((cur_elevator_ptr->pending_floor_call_bit_field & (1 << cur_floor)))
			   { 
			     ++stop_floor_call_in_car_and_move;
			   }
		  }
		  if(stop_floor_call_move >= stop_floor_call_in_car_and_move)
		  {
		  	  cur_elevator_ptr->elevator_status = MOVED_UP_STATIONARY;
		  	  #ifdef TRACE_FLOW
		  	     printf("TRA_42: STARTUP_STATIONARY -> MOVED_UP_STATIONARY \n"); 
		  	  #endif   
		  }
		  else
		  {
		  	   cur_elevator_ptr->elevator_status = MOVED_DOWN_STATIONARY;
		  	   #ifdef TRACE_FLOW
		  	     printf("TRA_43: STARTUP_STATIONARY -> MOVED_DOWN_STATIONARY \n"); 
		  	  #endif 
		  }
		break;
		default:
		   appl_status_flag = ERR_FORMAT_INVALID;
		   Error_or_Warning_Proc("11.22.11", ERROR_OCCURED, appl_status_flag);
		   return appl_status_flag;	
	 }
	return SUCCESS;
}

/*------------------------------------------------------------*
FUNCTION NAME  :  FSM_Abnormal_Event_Proc

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 14.24

BUGS           :              
-*------------------------------------------------------------*/
uint16_t FSM_Abnormal_Event_Proc(const uint8_t ctrl_elevator_ch_id)
{
	#ifdef TRACE_ERROR
    	printf("ERR: Abnormal event. Please Reset\n");
	#endif
    return FAILURE;
}

/*------------------------------------------------------------*
FUNCTION NAME  : Active_Hall_And_In_Car_Calls_Proc

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 11.21

BUGS           :              
-*------------------------------------------------------------*/
uint16_t Active_Hall_And_In_Car_Calls_Proc(const uint8_t ctrl_elevator_ch_id )
{
    elevator_ctrl_and_status_t *cur_elevator_ptr;
    uint16_t ret_status;
	uint32_t floor_call, floor_type;
	
	if(ctrl_elevator_ch_id >= MAX_NUM_ELEVATORS)
	{
		appl_status_flag = ERR_ELEVATOR_CH_ID_EXCEEDS;
	    Error_or_Warning_Proc("11.21.01", ERROR_OCCURED, appl_status_flag);
	    return appl_status_flag;
    }	
    cur_elevator_ptr = elevator_ctrl_and_status + ctrl_elevator_ch_id;
	switch(cur_elevator_ptr->before_fsm_state)
	{
		case FSM_STARTUP:
		  #ifdef TRACE_ERROR
		     printf("WARN: floor calls not allowed at startup \n");
		  #endif
		  appl_status_flag = ERR_FLOOR_CALLS_NOT_ALLOWED;
	      Error_or_Warning_Proc("11.21.02", WARNING_OCCURED, appl_status_flag);		 
		  return appl_status_flag;
	    //break;
        case FSM_PREPARE_USER_ENTRY_AND_EXIT:
         	printf("1 - in car, 2 - hall up, 3 - hall down \n");
			printf("Enter floor call type : ");
	        scanf("%u", &floor_type);
			switch(floor_type)
			{
		    	case IN_CAR_FLOOR_CALL_OPER:
			    case HALL_UP_FLOOR_CALL_OPER:
				case HALL_DOWN_FLOOR_CALL_OPER:
				    printf("Enter floor call : ");	
	                scanf("%u", &floor_call);
					if((ret_status = Validate_Floor(floor_call)) != SUCCESS)
					{
						appl_status_flag = ERR_FLOOR_INVALID;
	                    Error_or_Warning_Proc("11.21.03", ERROR_OCCURED, appl_status_flag);		 
		                return appl_status_flag;
					}
					if(cur_elevator_ptr->elevator_status == STARTUP_STATIONARY || cur_elevator_ptr->elevator_status == MOVED_UP_STATIONARY || cur_elevator_ptr->elevator_status == MOVED_DOWN_STATIONARY)
					{
					    if(floor_call == cur_elevator_ptr->cur_floor)
	                    {
	        	            #ifdef TRACE_ERROR 
		                       uint32_temp_disp_data = floor_call;
		                       printf("WARN: floor_call: %u = cur_floor \n", uint32_temp_disp_data);
                            #endif 		
	                        appl_status_flag = WARN_FLOOR_CALL_SAME_CUR_FLOOR;
	                        Error_or_Warning_Proc("11.21.04", WARNING_OCCURED, appl_status_flag);
	                        return appl_status_flag; 
	                   }
					}
				break;
				default:
				   printf("ERR: Invalid choice \n");
				   appl_status_flag = ERR_FORMAT_INVALID ;
	               Error_or_Warning_Proc("11.21.05", ERROR_OCCURED, appl_status_flag);		 
		           return appl_status_flag;
			}
			switch(floor_type) 
			{
				case IN_CAR_FLOOR_CALL_OPER:
			       if((cur_elevator_ptr->in_car_floor_call_bit_field & (1 << floor_call)))
			    	{
						 #ifdef TRACE_ERROR
		                      uint32_temp_disp_data = floor_call;
		                      printf("WARN: floor_call: %u was already be selected in_car \n", floor_call);
		                 #endif
				         appl_status_flag = WARN_IN_CAR_FLOOR_CALL_IS_ACTIVE ;
	                     Error_or_Warning_Proc("11.21.06", WARNING_OCCURED, appl_status_flag);		 
		                 return appl_status_flag;
				    } 						   					   
				    cur_elevator_ptr->in_car_floor_call_bit_field |= (1 << floor_call);
				    #ifdef TRACE_DATA
				       printf("TRA: in car floor calls        : 0x%X\n", cur_elevator_ptr->in_car_floor_call_bit_field);
			    	#endif
					if(cur_elevator_ptr->cur_max_floor_call_in_car == max_num_floors + 1)
	                {
	 	                cur_elevator_ptr->cur_max_floor_call_in_car = floor_call;
		                cur_elevator_ptr->cur_min_floor_call_in_car = floor_call;
					    cur_elevator_ptr->num_pending_calls_in_car = 1; 
	                }
	                else
	                {
	        	         if(cur_elevator_ptr->cur_max_floor_call_in_car < floor_call)
		                 {
	 	                	cur_elevator_ptr->cur_max_floor_call_in_car = floor_call;
	                     }
		                 if(cur_elevator_ptr->cur_min_floor_call_in_car > floor_call)
		                 {
		                	cur_elevator_ptr->cur_min_floor_call_in_car = floor_call;
		                 }
						 ++cur_elevator_ptr->num_pending_calls_in_car;
		                 #ifdef TRACE_DATA
		                    uint32_temp_disp_data = cur_elevator_ptr->cur_min_floor_call_in_car;
		                    printf("TRA: pending floor call range[%u , ", uint32_temp_disp_data);
		                    uint32_temp_disp_data = cur_elevator_ptr->cur_max_floor_call_in_car;
		        	        printf("%u], ", uint32_temp_disp_data); 
							uint32_temp_disp_data = cur_elevator_ptr->num_pending_calls_in_car;
		        	        printf("num calls: %u\n", uint32_temp_disp_data);
		                 #endif	 
	               }
			   break;
		       case HALL_UP_FLOOR_CALL_OPER:
				   if(floor_call >= max_num_floors - 1)
					{
					   appl_status_flag = ERR_HALL_CALL_UP_FLOOR_INVALID ;
	                   Error_or_Warning_Proc("11.21.07", WARNING_OCCURED, appl_status_flag);		 
		               return appl_status_flag; 
					}	
					if((cur_elevator_ptr->hall_floor_call_up_bit_field & (1 << floor_call)))
					{
						#ifdef TRACE_ERROR
		                      uint32_temp_disp_data = floor_call;
		                      printf("WARN: floor_call: %u was already be selected hall_up \n", floor_call);
		               #endif
					   appl_status_flag = WARN_HALL_CALL_UP_FLOOR_IS_ACTIVE ;
	                   Error_or_Warning_Proc("11.21.08", WARNING_OCCURED, appl_status_flag);		 
		               return appl_status_flag;
					}					   
					cur_elevator_ptr->hall_floor_call_up_bit_field |= (1 << floor_call);
					#ifdef TRACE_DATA
	                   printf("TRA: hall floor calls up        : 0x%X\n", cur_elevator_ptr->hall_floor_call_up_bit_field);
					#endif
					if(cur_elevator_ptr->cur_max_floor_call_up == max_num_floors + 1)
	                {
	 	               cur_elevator_ptr->cur_max_floor_call_up = floor_call;
		               cur_elevator_ptr->cur_min_floor_call_up = floor_call;
					   cur_elevator_ptr->num_pending_calls_up = 1;    
	                }
	                else
	                {
	        	         if(cur_elevator_ptr->cur_max_floor_call_up < floor_call)
		                 {
	 	                	cur_elevator_ptr->cur_max_floor_call_up = floor_call;
	                     }
		                 if(cur_elevator_ptr->cur_min_floor_call_up > floor_call)
		                 {
		                	cur_elevator_ptr->cur_min_floor_call_up = floor_call;
		                 }
						 ++cur_elevator_ptr->num_pending_calls_up;
		                 #ifdef TRACE_DATA
		                    uint32_temp_disp_data = cur_elevator_ptr->cur_min_floor_call_up;
		                    printf("TRA: pending floor call range[%u , ", uint32_temp_disp_data);
		                    uint32_temp_disp_data = cur_elevator_ptr->cur_max_floor_call_up;
		        	        printf("%u], ", uint32_temp_disp_data); 
							uint32_temp_disp_data = cur_elevator_ptr->num_pending_calls_up;
		        	        printf("num calls: %u\n", uint32_temp_disp_data);
		                 #endif	 
	               }
			   break;
			   case HALL_DOWN_FLOOR_CALL_OPER:
					  if(floor_call == FLOOR_00)
					   {
					      appl_status_flag = ERR_HALL_CALL_DOWN_FLOOR_INVALID ;
	                      Error_or_Warning_Proc("11.21.09", WARNING_OCCURED, appl_status_flag);		 
		                  return appl_status_flag; 
					   }	
					   if((cur_elevator_ptr->hall_floor_call_down_bit_field & (1 << floor_call)))
					   {
						   #ifdef TRACE_ERROR
		                      uint32_temp_disp_data = floor_call;
		                      printf("WARN: floor_call: %u was already be selected hall_down \n", floor_call);
		                   #endif
					       appl_status_flag = WARN_HALL_CALL_DOWN_FLOOR_IS_ACTIVE ;
	                       Error_or_Warning_Proc("11.21.10", WARNING_OCCURED, appl_status_flag);		 
		                   return appl_status_flag;
					   }				   
					   cur_elevator_ptr->hall_floor_call_down_bit_field |= (1 << floor_call);
					   #ifdef TRACE_DATA 
					         printf("TRA: hall floor calls down       : 0x%X\n", cur_elevator_ptr->hall_floor_call_down_bit_field);
					   #endif
					   if(cur_elevator_ptr->cur_max_floor_call_down == max_num_floors + 1)
	                   {
	 	                 cur_elevator_ptr->cur_max_floor_call_down = floor_call;
		                 cur_elevator_ptr->cur_min_floor_call_down = floor_call;
					     cur_elevator_ptr->num_pending_calls_down = 1; 
	                   }
	                   else
	                   {
	                   	 ++cur_elevator_ptr->num_pending_calls_down;
	        	         if(cur_elevator_ptr->cur_max_floor_call_down < floor_call)
		                 {
	 	                	cur_elevator_ptr->cur_max_floor_call_down = floor_call;
	                     }
		                 if(cur_elevator_ptr->cur_min_floor_call_down > floor_call)
		                 {
		                	cur_elevator_ptr->cur_min_floor_call_down = floor_call;
		                 }						 
		                 #ifdef TRACE_DATA
		                    uint32_temp_disp_data = cur_elevator_ptr->cur_min_floor_call_down;
		                    printf("TRA: pending floor call range[%u , ", uint32_temp_disp_data);
		                    uint32_temp_disp_data = cur_elevator_ptr->cur_max_floor_call_down;
		        	        printf("%u], ", uint32_temp_disp_data); 
							uint32_temp_disp_data = cur_elevator_ptr->num_pending_calls_down;
		        	        printf("num calls: %u\n", uint32_temp_disp_data);
		                 #endif	 
	                  }
				break;    					
			}
			if((cur_elevator_ptr->pending_floor_call_bit_field & (1 << floor_call)))
			{
				return SUCCESS;
			}
	       cur_elevator_ptr->pending_floor_call_bit_field |= (1 << floor_call);
	       #ifdef TRACE_INFO
	          uint32_temp_disp_data = floor_call; 
	          printf("INFO: Pending floor call : %u added \n", uint32_temp_disp_data);
           #endif	
	       if(cur_elevator_ptr->cur_max_floor_call == max_num_floors + 1)
	       {
	 	       cur_elevator_ptr->cur_max_floor_call = floor_call;
		       cur_elevator_ptr->cur_min_floor_call = floor_call;
		       cur_elevator_ptr->next_stop_floor = floor_call;
			   cur_elevator_ptr->num_pending_calls = 1;
		       #ifdef TRACE_DATA
		            uint32_temp_disp_data = floor_call;
		            printf("TRA: first pending, next stop floor : %u\n", uint32_temp_disp_data);
	           #endif		  
	       }
	       else
	       {
			     ++cur_elevator_ptr->num_pending_calls;
	        	 if(cur_elevator_ptr->cur_max_floor_call < floor_call)
		         {
	 	        	cur_elevator_ptr->cur_max_floor_call = floor_call;
	             }
		         if(cur_elevator_ptr->cur_min_floor_call > floor_call)
		         {
		        	cur_elevator_ptr->cur_min_floor_call = floor_call;
		         }
		         #ifdef TRACE_DATA
		            uint32_temp_disp_data = cur_elevator_ptr->cur_min_floor_call;
		            printf("TRA: pending floor call range[%u , ", uint32_temp_disp_data);
		            uint32_temp_disp_data = cur_elevator_ptr->cur_max_floor_call;
		        	printf("%u] \n", uint32_temp_disp_data); 
					uint32_temp_disp_data = cur_elevator_ptr->num_pending_calls;
		        	printf("num calls: %u\n", uint32_temp_disp_data);
		        #endif	 
	       }
	      #ifdef TRACE_INFO
	          uint32_temp_disp_data = cur_elevator_ptr->pending_floor_call_bit_field;
	          printf("TRA: pending floor calls : 0x%x \n", uint32_temp_disp_data);
	      #endif
	    break;
	}
	return SUCCESS;				
}

/*------------------------------------------------------------*
FUNCTION NAME  : Disp_Oper_Proc

DESCRIPTION    :
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 11.11

BUGS           :    
-*------------------------------------------------------------*/ 
uint16_t Disp_Oper_Proc(const uint8_t ctrl_elevator_ch_id )
{
	 elevator_ctrl_and_status_t *cur_elevator_ptr;
	 cur_elevator_ptr = elevator_ctrl_and_status + ctrl_elevator_ch_id;
	           printf("Num floors                    : %u\n", max_num_floors);
			   if(cur_elevator_ptr->cur_floor < max_num_floors)
			   {
	               printf("Current floor                 : %u\n", cur_elevator_ptr->cur_floor);
			   }
			   else
			   {
				   printf("Current floor                 : -\n");
			   }
			   if(cur_elevator_ptr->next_stop_floor < max_num_floors)
			   {
	              printf("Next stop floor               : %u\n", cur_elevator_ptr->next_stop_floor);
			   }
		       else
			   {
				  printf("Next stop floor               : -\n");
			   }
			   printf("Pending floor calls - bit     : 0x%X\n", cur_elevator_ptr->pending_floor_call_bit_field);
			   printf("Num pending calls             : %u\n", cur_elevator_ptr->num_pending_calls);
			   if(cur_elevator_ptr->cur_min_floor_call < max_num_floors)
			   {
	              printf("Min_stop_floor                : %u\n", cur_elevator_ptr->cur_min_floor_call);
			   }
			   else
			   {
                  printf("Min stop floor                : -\n");
			   }
               if(cur_elevator_ptr->cur_max_floor_call < max_num_floors)	
			   {				   
	              printf("Max stop floor                : %u\n", cur_elevator_ptr->cur_max_floor_call);
			   }
			   else
			   {
				  printf("Max stop floor                : -\n");
			   }				  
			   printf("In car floor calls - bit      : 0x%X\n", cur_elevator_ptr->in_car_floor_call_bit_field);
			   printf("Num pending in car calls      : %u\n", cur_elevator_ptr->num_pending_calls_in_car);
			   if(cur_elevator_ptr->cur_min_floor_call_in_car < max_num_floors)
			   {
			       printf("Min stop floor in car         : %u\n", cur_elevator_ptr->cur_min_floor_call_in_car);
			   }
			   else
			   {
				   printf("Min stop floor in car         : -\n", cur_elevator_ptr->cur_min_floor_call_in_car); 
			   }
			   if(cur_elevator_ptr->cur_max_floor_call_in_car <= max_num_floors)
			   {
	              printf("Max stop floor in car         : %u\n", cur_elevator_ptr->cur_max_floor_call_in_car);
			   }
               else
			   {
                  printf("Max stop floor in car         : -\n"); 	
			   }				  
			   printf("Hall floor calls up           : 0x%X\n", cur_elevator_ptr->hall_floor_call_up_bit_field);
			   printf("Num pending up calls          : %u\n", cur_elevator_ptr->num_pending_calls_up);
			   if(cur_elevator_ptr->cur_min_floor_call_up < max_num_floors)
			   {
	              printf("Min_stop_floor_up             : %u\n", cur_elevator_ptr->cur_min_floor_call_up);
			   }
			   else
			   {
				  printf("Min stop floor up             : -\n");
			   }
               if(cur_elevator_ptr->cur_max_floor_call_up < max_num_floors)	
			   {				   
	              printf("Max stop floor up             : %u\n", cur_elevator_ptr->cur_max_floor_call_up);
			   }
			   else
			   {
				  printf("Max stop floor up             : -\n", cur_elevator_ptr->cur_max_floor_call_up); 
			   }				  
			   printf("Hall floor calls down - bit   : 0x%X\n", cur_elevator_ptr->hall_floor_call_down_bit_field);
			   printf("Num pending down calls        : %u\n", cur_elevator_ptr->num_pending_calls_down);
			   if(cur_elevator_ptr->cur_min_floor_call_down < max_num_floors)
			   {
	               printf("Min stop floor down           : %u\n", cur_elevator_ptr->cur_min_floor_call_down);
			   }
			   else
			   {
				   printf("Min stop floor down           : -\n"); 
			   }
			   if(cur_elevator_ptr->cur_max_floor_call_down < max_num_floors)
			   {
				   printf("Max stop floor down           : %u\n", cur_elevator_ptr->cur_max_floor_call_down);
			   }
			   else
			   {
				   printf("Max stop floor down           : -\n"); 
			   }
	return SUCCESS;
}
/*------------------------------------------------------------*
FUNCTION NAME  : Car_Movement_Direction 

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 11.16

BUGS           :              
-*------------------------------------------------------------*/
uint16_t Car_Movement_Direction(const uint8_t ctrl_elevator_ch_id, const uint8_t cur_floor, uint8_t *const elevator_trigger_move_ptr)
{
	elevator_ctrl_and_status_t *cur_elevator_ptr;
	
	if(elevator_trigger_move_ptr == NULL_PTR)
	{
		appl_status_flag = ERR_NULL_PTR;
	    Error_or_Warning_Proc("11.16.01", ERROR_OCCURED, appl_status_flag);
	    return appl_status_flag;
	}
	*elevator_trigger_move_ptr = NUM_ELEVATOR_STATUS;
	if(ctrl_elevator_ch_id >= MAX_NUM_ELEVATORS)
	{
		appl_status_flag = ERR_ELEVATOR_CH_ID_EXCEEDS;
	    Error_or_Warning_Proc("11.16.02", ERROR_OCCURED, appl_status_flag);
	    return appl_status_flag;
    }	
    cur_elevator_ptr = elevator_ctrl_and_status + ctrl_elevator_ch_id;	
	
    if(cur_floor == cur_elevator_ptr->next_stop_floor)
	{
		switch(cur_elevator_ptr->elevator_status)
		{
			case STARTUP_STATIONARY:
			   *elevator_trigger_move_ptr = STARTUP_STATIONARY;
			break;
			case MOVE_UP:
			case MOVED_UP_STATIONARY:
			   *elevator_trigger_move_ptr = MOVED_UP_STATIONARY;
			break;
			case MOVE_DOWN:
			case MOVED_DOWN_STATIONARY:
			   *elevator_trigger_move_ptr = MOVED_DOWN_STATIONARY;
			break;
            default:
               appl_status_flag = ERR_FORMAT_INVALID;
	           Error_or_Warning_Proc("11.16.03", ERROR_OCCURED, appl_status_flag);
	           return appl_status_flag;
		}			   
	}
	else
	{
		if(cur_floor < cur_elevator_ptr->next_stop_floor) 
	    {			
			 *elevator_trigger_move_ptr = MOVE_UP;
		}
		else
		{	
	         *elevator_trigger_move_ptr = MOVE_DOWN;		   
		}	
	}
	return SUCCESS;
}


/*------------------------------------------------------------*
FUNCTION NAME  : Appl_Reset_Proc

DESCRIPTION    :
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 11.11

BUGS           :    
-*------------------------------------------------------------*/ 
uint16_t Appl_Reset_Proc(const uint8_t cur_ctrl_elevator_ch_id)
{
	int32_t enter_num_floors;
	
	if(cur_ctrl_elevator_ch_id >= MAX_NUM_ELEVATORS)
	{
		appl_status_flag = ERR_ELEVATOR_CH_ID_EXCEEDS;
	    Error_or_Warning_Proc("11.11.01", ERROR_OCCURED, appl_status_flag);
	    return appl_status_flag;
    } 
	printf("Enter Num floors: ");
	scanf("%d", &enter_num_floors);
	if(enter_num_floors < MIN_NUM_FLOORS || enter_num_floors > MAX_NUM_FLOORS)
	{
		printf("ERR: num_floor entered: %u invalid, valid range[ %u, %u ]\n",enter_num_floors, MIN_NUM_FLOORS, MAX_NUM_FLOORS );
		printf("Considered Num floors: %u\n", max_num_floors);		
	}
	else
	{
	   max_num_floors = (uint32_t)enter_num_floors;
	   #ifdef TRACE_INFO
    	   printf("Entered num floors:  %u\n", max_num_floors);
	   #endif
	}
    Reset_Elevator_Datas(cur_ctrl_elevator_ch_id, RESET_WHOLE_DATAS);
	return SUCCESS;
}	


/*------------------------------------------------------------*
FUNCTION NAME  : Validate_Floor

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 11.12  

BUGS           :              
-*------------------------------------------------------------*/
uint16_t Validate_Floor(const uint8_t floor)
{
	
	if(floor < 0 || floor >=  max_num_floors)
	{
		#ifdef TRACE_ERROR
		   printf("ERR: Floor: %u is invalid: valid_range[ 0 , %u ] \n", floor, max_num_floors - 1);
		#endif
		appl_status_flag = ERR_FLOOR_INVALID;
		Error_or_Warning_Proc("11.12.01", ERROR_OCCURED, appl_status_flag);
		return appl_status_flag;
	}
	return SUCCESS;
}


/*------------------------------------------------------------*
FUNCTION NAME  : Reset_Elevator_Datas

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 11.12  

BUGS           :              
-*------------------------------------------------------------*/
uint16_t Reset_Elevator_Datas(const uint8_t ctrl_elevator_ch_id, const uint8_t reset_type)
{
    elevator_ctrl_and_status_t *cur_elevator_ptr; 
    if(ctrl_elevator_ch_id >= MAX_NUM_ELEVATORS)
	{
		appl_status_flag = ERR_ELEVATOR_CH_ID_EXCEEDS;
	    Error_or_Warning_Proc("11.22.02", ERROR_OCCURED, appl_status_flag);
	    return appl_status_flag;
    }	
    cur_elevator_ptr = elevator_ctrl_and_status + ctrl_elevator_ch_id;	
	switch(reset_type)
	{
		case RESET_WHOLE_DATAS:
	       cur_elevator_ptr->cur_fsm_state = FSM_STARTUP;
	       cur_elevator_ptr->before_fsm_state = FSM_STARTUP;
	       cur_elevator_ptr->elevator_status = STARTUP_STATIONARY;
		   cur_elevator_ptr->cur_floor =  max_num_floors + 1;
	    case RESET_STOP_DATAS:
	       cur_elevator_ptr->next_stop_floor = max_num_floors + 1;
	       cur_elevator_ptr->cur_min_floor_call = max_num_floors + 1;
	       cur_elevator_ptr->cur_max_floor_call = max_num_floors + 1;
	       cur_elevator_ptr->cur_min_floor_call_in_car = max_num_floors + 1;
	       cur_elevator_ptr->cur_max_floor_call_in_car = max_num_floors + 1;
	       cur_elevator_ptr->cur_min_floor_call_up = max_num_floors + 1;
	       cur_elevator_ptr->cur_max_floor_call_up = max_num_floors + 1;	
	       cur_elevator_ptr->cur_min_floor_call_down = max_num_floors + 1;
	       cur_elevator_ptr->cur_max_floor_call_down = max_num_floors + 1;	
	       cur_elevator_ptr->pending_floor_call_bit_field = 0;
	       cur_elevator_ptr->in_car_floor_call_bit_field = 0;
           cur_elevator_ptr->hall_floor_call_up_bit_field = 0;
           cur_elevator_ptr->hall_floor_call_down_bit_field = 0;
	       cur_elevator_ptr->num_pending_calls_in_car = 0;
           cur_elevator_ptr->num_pending_calls_up = 0;
           cur_elevator_ptr->num_pending_calls_down = 0;
	       cur_elevator_ptr->num_pending_calls = 0;	 	 
	   break;
       default:
	      appl_status_flag = ERR_FORMAT_INVALID;
	      Error_or_Warning_Proc("11.22.03", ERROR_OCCURED, appl_status_flag);
	      return appl_status_flag;
	}
	return SUCCESS;
}

/*------------------------------------------------------------*
FUNCTION NAME  : Error_or_Warning_Proc

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 02.14  

BUGS           :              
-*------------------------------------------------------------*/
uint8_t Error_or_Warning_Proc(const char *const error_trace_str, const uint8_t warn_or_error_format, const uint32_t warning_or_error_code)
{
	switch(warn_or_error_format)
	{
		case WARNING_OCCURED:
			printf("WARN: "); 
		break;
		default:
		   printf("ERR: ");	
	}
	printf("%s, code: %u \n", error_trace_str, warning_or_error_code);
	return SUCCESS;
}

/*------------------------------------------------------------*
FUNCTION NAME  : Appl_Reset

DESCRIPTION    :
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 11.01

BUGS           :    
-*------------------------------------------------------------*/
 uint16_t Appl_Reset(const uint8_t reset_type)
{
	uint16_t ret_status;
	
	switch(reset_type)
	{
		case RESET_APPL:	
           appl_status_flag = NO_ERROR;
           if((ret_status = Appl_Reset_Proc(CTRL_ELEVATOR_CH_ID)) != SUCCESS)
		   {
			   return FAILURE;
		   }			   
		break;
		default:
		   appl_status_flag = ERR_FORMAT_INVALID;
		   Error_or_Warning_Proc("11.01.04", ERROR_OCCURED, appl_status_flag);
		   return appl_status_flag;
	}
	return SUCCESS;
}


