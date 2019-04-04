/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
*
* Use of this source code is restricted per the CC BY-NC-ND license, a copy of
* which can be found via http://creativecommons.org (and should be included as
* LICENSE.txt within the associated archive or repository).
*/

#include "hilevel.h"

#define maxProcesses 32

pcb_t pcb[ maxProcesses ];
pcb_t* current = NULL;
extern uint32_t tos_process_stack; //Extern declares but doesn't define / allocate mem
extern uint32_t tos_console;
extern void main_console();
int process_size_stack_offset = 0x1000;


void dispatch( ctx_t* ctx, pcb_t* prev, pcb_t* next ) {
  char prev_pid = '?', next_pid = '?';

  if( NULL != prev ) {
    memcpy( &prev->ctx, ctx, sizeof( ctx_t ) ); // preserve execution context of P_{prev}
    prev_pid = '0' + prev->pid;
  }
  if( NULL != next ) {
    memcpy( ctx, &next->ctx, sizeof( ctx_t ) ); // restore  execution context of P_{next}
    next_pid = '0' + next->pid;
  }

  PL011_putc( UART0, '[',      true );
  PL011_putc( UART0, prev_pid, true );
  PL011_putc( UART0, '-',      true );
  PL011_putc( UART0, '>',      true );
  PL011_putc( UART0, next_pid, true );
  PL011_putc( UART0, ']',      true );

  current = next;                             // update   executing index   to P_{next}
  prev->status = STATUS_READY;
  next->status = STATUS_EXECUTING;
  return;
}

//Round robin
void round_robin_schedule( ctx_t* ctx ) {
  if     ( current->pid == pcb[ 0 ].pid ) {
    dispatch( ctx, &pcb[ 0 ], &pcb[ 1 ] );      // context switch P_3 -> P_4

    pcb[ 0 ].status = STATUS_READY;             // update   execution status  of P_3
    pcb[ 1 ].status = STATUS_EXECUTING;         // update   execution status  of P_4
  }
  else if( current->pid == pcb[ 1 ].pid ) {
    dispatch( ctx, &pcb[ 1 ], &pcb[ 2 ] );      // context switch P_4 -> P_5

    pcb[ 1 ].status = STATUS_READY;             // update   execution status  of P_4
    pcb[ 2 ].status = STATUS_EXECUTING;         // update   execution status  of P_5
  } else if ( current->pid == pcb[2].pid){
    dispatch( ctx, &pcb[2], &pcb[0]);           // context switch P_5 -> P_3

    pcb[ 2 ].status = STATUS_READY;             // update   execution status  of P_5
    pcb[ 0 ].status = STATUS_EXECUTING;         // update   execution status  of P_3
  }

  return;
}

int getCurrentIndex(){
  return (current->pid);
}


//Priority queue schedule
void priority_queue_schedule( ctx_t* ctx ) {
  int highest_priority = 0;
  int highest_priority_index = 0;
  int currentIndex = getCurrentIndex();
  for (int i = 0; i < maxProcesses; i++){
    if (pcb[i].status != STATUS_TERMINATED){ //Only work on 'live' processes
      if ( pcb[i].priority > highest_priority ) {
        highest_priority = pcb[i].priority;
        highest_priority_index = i;
      }
      //Age the non-current processes
      if (getCurrentIndex() != i){
        pcb[i].priority += 1;
      }
    }
  }
  if ( highest_priority > current->priority ){
    //Reset the age
    pcb[highest_priority_index].priority = pcb[highest_priority_index].base_priority;
    //Swap to next process
    dispatch(ctx, &pcb[currentIndex], &pcb[highest_priority_index]);
    pcb[currentIndex].status = STATUS_READY;
    pcb[highest_priority_index].status = STATUS_EXECUTING;
  }
  return;
}


extern void     main_P3();
extern uint32_t tos_P3;
extern void     main_P4();
extern uint32_t tos_P4;
extern void     main_P5();
extern uint32_t tos_P5;

uint32_t get_stack_address_for_process(pid_t pid){
  uint32_t top_address = (uint32_t) (&tos_process_stack);
  uint32_t process_stack_address = top_address - (process_size_stack_offset * pid);
  return process_stack_address;
}


void hilevel_handler_rst( ctx_t* ctx              ) {
 
	// Initliase console process
  memset( &pcb[ 0 ], 0, sizeof( pcb_t ) );
  pcb[ 0 ].pid      = 0;
  pcb[ 0 ].status   = STATUS_CREATED;
  pcb[ 0 ].ctx.cpsr = 0x50;
  pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
  pcb[ 0 ].ctx.sp   = ( uint32_t )( &tos_process_stack  );
  pcb[ 0 ].base_priority = 7;
  pcb[ 0 ].priority = 7;
  

  //Initliase the remaining empty/blank pcbs
  for (int i = 1; i < maxProcesses; i ++){
    memset( &pcb[ i ], 0, sizeof( pcb_t ) );     // initialise ith PCB = P_i
    pcb[ i ].pid      = i;
    pcb[ i ].status   = STATUS_TERMINATED;
    pcb[ i ].ctx.cpsr = 0x50;
    pcb[ i ].ctx.pc   = ( uint32_t )( &main_console ); //WHATTHEHELLISTHIS
    pcb[ i ].ctx.sp   = ( uint32_t )( get_stack_address_for_process(i)  );
    pcb[ i ].base_priority = 0;
    pcb[ i ].priority = 0;
  }

	//Initalise timer
  TIMER0->Timer1Load  = 0x00100000; // select period = 2^20 ticks ~= 1 sec
  TIMER0->Timer1Ctrl  = 0x00000002; // select 32-bit   timer
  TIMER0->Timer1Ctrl |= 0x00000040; // select periodic timer
  TIMER0->Timer1Ctrl |= 0x00000020; // enable          timer interrupt
  TIMER0->Timer1Ctrl |= 0x00000080; // enable          timer

	//Initalise the GIC (interupt handler)
  GICC0->PMR          = 0x000000F0; // unmask all            interrupts
  GICD0->ISENABLER1  |= 0x00000010; // enable timer          interrupt
  GICC0->CTLR         = 0x00000001; // enable GIC interface
  GICD0->CTLR         = 0x00000001; // enable GIC distributor

	//Dispatch/execute the console
  dispatch( ctx, NULL, &pcb[ 0 ] );
  int_enable_irq();

  return;
}

void hilevel_handler_irq(ctx_t* ctx) {
  // Step 2: read  the interrupt identifier so we know the source.

  uint32_t id = GICC0->IAR;

  // Step 4: handle the interrupt, then clear (or reset) the source.

  if( id == GIC_SOURCE_TIMER0 ) {
    priority_queue_schedule( ctx ); TIMER0->Timer1IntClr = 0x01;
  }

  // Step 5: write the interrupt identifier to signal we're done.

  GICC0->EOIR = id;

  return;

}

pcb_t* get_empty_pcb(){
	for (int i = 0; i < maxProcesses; i++){
		if (pcb[i].status == STATUS_TERMINATED) {
			return &pcb[i];
		}
	}
}

void terminate_process(pcb_t* process, ctx_t* ctx){
	process->status = STATUS_TERMINATED;
	//Since the current program is now terminated, we need to run another by calling the scheduler
	priority_queue_schedule(ctx);
}

void sys_write(ctx_t* ctx){
	int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      for( int i = 0; i < n; i++ ) {
        PL011_putc( UART0, *x++, true );
      }

      ctx->gpr[ 0 ] = n;
}

void sys_read(ctx_t* ctx){
	int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      for( int i = 0; i < n; i++ ) {
        x[i] = PL011_getc(UART0, true);
      }

      ctx->gpr[ 0 ] = n;
}

void sys_fork( ctx_t* ctx){
	pcb_t* child = get_empty_pcb(); //add check for no empty???
	memcpy( &child->ctx, ctx, sizeof( ctx_t ) ); //Copy current context into child
	child->status = STATUS_READY;
	child->base_priority = 1;
	child->priority = 1;
	//Put stack pointer into the child stack space, taking into account how far 'in' the pointer is
	child->ctx.sp = get_stack_address_for_process(child->pid) - get_stack_address_for_process(current->pid) + ctx->sp;
	//Copy accross stack
	memcpy((void*)(child->ctx.sp), (void*) (ctx->sp), process_size_stack_offset);
	//Return 0 to child
	child->ctx.gpr[0] = 0; 
	//Return child PID to parent
	ctx->gpr[0] = child->pid;
}

void sys_exit(ctx_t* ctx){
	terminate_process(current, ctx);
}

void sys_exec(ctx_t* ctx){
	PL011_putc( UART0, 'E', true );
	//Address of 'new' program is stored in gpr[0]
	ctx->pc = ctx->gpr[0];
	//Set stack pointer to start, incase sp is halfway through, might not be needed
	//ctx->sp = get_stack_address_for_process(getCurrentIndex);
}

void hilevel_handler_svc( ctx_t* ctx, uint32_t id ) {
  /* Based on the identifier (i.e., the immediate operand) extracted from the
  * svc instruction,
  *
  * - read  the arguments from preserved usr mode registers,
  * - perform whatever is appropriate for this system call, then
  * - write any return value back to preserved usr mode registers.
  */
  switch( id ) {
    case 0x00 : { // 0x00 => YIELD()
      priority_queue_schedule( ctx );
      break;
    }
    case 0x01 : { // 0x01 => WRITE( fd, x, n )
      sys_write(ctx);
      break;
    }
    case 0x02 :{ // 0x02 => READ( fd, x, n )
      sys_read(ctx);
      break;
	}
    case 0x03:{ // 0x03 => FORK(parent, child)
		sys_fork(ctx);
		break;
    }
	case 0x04:{ //0x04 => EXIT(process)
		sys_exit(ctx);
		break;
	}
	  case 0x05:{ //0x05 => EXEC
		  sys_exec(ctx);
		  break;
	  }
  default   : { // 0x?? => unknown/unsupported
    break;
  }
}
return;
}
