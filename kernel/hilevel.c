/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

pcb_t pcb[ 3 ]; pcb_t* current = NULL;

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

  return;
}

//Round robin
void schedule( ctx_t* ctx ) {
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
  for (int i = 0; i < 3; i++){
    if (current->pid == pcb[i].pid){
      return i;
    }
  }
  return 0;
}

//Priority queue schedule
void priority_queue_schedule( ctx_t* ctx ) {
  int highest_priority = 0;
  int highest_priority_pid = 0;
  int highest_priority_index = 0;
  for (int i = 0; i < 3; i++){
    if ( pcb[i].priority > highest_priority ) {
      highest_priority = pcb[i].priority;
      highest_priority_pid = pcb[i].pid;
    }
  }
  if (current->priority  < highest_priority ){
    int currentIndex = *getCurrentIndex;
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

void hilevel_handler_rst( ctx_t* ctx              ) {
  /* Initialise two PCBs, representing user processes stemming from execution
   * of two user programs.  Note in each case that
   *
   * - the CPSR value of 0x50 means the processor is switched into USR mode,
   *   with IRQ interrupts enabled, and
   * - the PC and SP values matche the entry point and top of stack.
   */

  memset( &pcb[ 0 ], 0, sizeof( pcb_t ) );     // initialise 0-th PCB = P_3
  pcb[ 0 ].pid      = 3;
  pcb[ 0 ].status   = STATUS_CREATED;
  pcb[ 0 ].ctx.cpsr = 0x50;
  pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_P3 );
  pcb[ 0 ].ctx.sp   = ( uint32_t )( &tos_P3  );
  pcb[ 0 ].priority = 6;

  memset( &pcb[ 1 ], 0, sizeof( pcb_t ) );     // initialise 1-st PCB = P_4
  pcb[ 1 ].pid      = 4;
  pcb[ 1 ].status   = STATUS_CREATED;
  pcb[ 1 ].ctx.cpsr = 0x50;
  pcb[ 1 ].ctx.pc   = ( uint32_t )( &main_P4 );
  pcb[ 1 ].ctx.sp   = ( uint32_t )( &tos_P4  );
  pcb[ 1 ].priority = 6;

  memset( &pcb[ 2 ], 0, sizeof( pcb_t ) );     // initialise 1-st PCB = P_5
  pcb[ 2 ].pid      = 5;
  pcb[ 2 ].status   = STATUS_CREATED;
  pcb[ 2 ].ctx.cpsr = 0x50;
  pcb[ 2 ].ctx.pc   = ( uint32_t )( &main_P5 );
  pcb[ 2 ].ctx.sp   = ( uint32_t )( &tos_P5  );
  pcb[ 2 ].priority = 10;

  /* Once the PCBs are initialised, we arbitrarily select the one in the 0-th
   * PCB to be executed: there is no need to preserve the execution context,
   * since it is is invalid on reset (i.e., no process will previously have
   * been executing).
   */


  TIMER0->Timer1Load  = 0x00100000; // select period = 2^20 ticks ~= 1 sec
  TIMER0->Timer1Ctrl  = 0x00000002; // select 32-bit   timer
  TIMER0->Timer1Ctrl |= 0x00000040; // select periodic timer
  TIMER0->Timer1Ctrl |= 0x00000020; // enable          timer interrupt
  TIMER0->Timer1Ctrl |= 0x00000080; // enable          timer

  GICC0->PMR          = 0x000000F0; // unmask all            interrupts
  GICD0->ISENABLER1  |= 0x00000010; // enable timer          interrupt
  GICC0->CTLR         = 0x00000001; // enable GIC interface
  GICD0->CTLR         = 0x00000001; // enable GIC distributor

  dispatch( ctx, NULL, &pcb[ 0 ] );
  int_enable_irq();

  return;
}

void hilevel_handler_irq(ctx_t* ctx) {
  // Step 2: read  the interrupt identifier so we know the source.

  uint32_t id = GICC0->IAR;

  // Step 4: handle the interrupt, then clear (or reset) the source.

  if( id == GIC_SOURCE_TIMER0 ) {
    schedule( ctx ); TIMER0->Timer1IntClr = 0x01;
  }

  // Step 5: write the interrupt identifier to signal we're done.

  GICC0->EOIR = id;

  return;
}

void hilevel_handler_svc() {
  return;
}
