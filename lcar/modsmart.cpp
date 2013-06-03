/* Rev:$Revision: 1.6 $ */
/******************************************************************************
 * Copyright 2005-2009 NetBurner, Inc  ALL RIGHTS RESERVED
 *   Permission is hereby granted to purchasers of NetBurner Hardware
 *   to use or modify this computer program for any use as long as the
 *   resultant program is only executed on NetBurner provided hardware.
 *
 *   No other rights to use this program or it's derivitives in part or
 *   in whole are granted.
 *
 *   It may be possible to license this or other NetBurner software for
 *   use on non NetBurner Hardware. Please contact sales@netburner.com
 *   for more infomation.
 *
 *   NetBurner makes no representation or warranties with respect to
 *   the performance of this computer program, and specifically disclaims
 *   any responsibility for any damages, special or consequential,
 *   connected with the use of this program.
 *
 *   NetBurner, Inc.
 *   5405 Morehouse Dr
 *   San Diego CA, 92121
 *   USA
 *****************************************************************************/
#include "predef.h"
#include <basictypes.h>
#include <constants.h>
#include <bsp.h>
#include <ucos.h>
#include <smarttrap.h>
#include "mytrap.h"
typedef struct {
    unsigned long table[255];
} vectors;

extern vectors vector_base;

static DWORD smart_trap_a7;
static DWORD smart_trap_stack[256];
volatile static BOOL already_in_trap;

extern "C" {
void smart_trap_asm();
void LocalOutByte(char c);
void Local_OutString(const char * cp);
void Local_OutHex(DWORD val, int bytes);
void Local_OutHexLong(DWORD val);

extern void UCOSWAITS_HERE(void);
}

extern volatile Captured_Trap cap_trap;


extern char __stack;
#define _heapend (& __stack)

void Local_OutString(const char * cp)
{
    while ( *cp )
        LocalOutByte( *cp++ );

}

void Local_OutHex(DWORD val, int bytes)
{

    PBYTE pb = (PBYTE) &val;
    for ( int i = bytes; i > 0; i-- )
    {
        BYTE b = ( pb[4 - i] );
        b = b >> 4;
        if ( b >= 10 )
            LocalOutByte( 'A' + ( b - 10 ) );
        else
            LocalOutByte( '0' + b );
        b = ( pb[4 - i] ) & 0x0f;
        if ( b >= 10 )
            LocalOutByte( 'A' + ( b - 10 ) );
        else
            LocalOutByte( '0' + b );
    }
}

void Local_OutHexLong(DWORD val)
{
    Local_OutHex( val, 4 );
}

void Local_ShowTaskName(BYTE task)
{
    switch ( task ) {
    case 0x3F:
        Local_OutString( "Idle," );
        break;
    case HTTP_PRIO:
        Local_OutString( "HTTP," );
        break;
    case NET_PRIO:
        Local_OutString( "NET ," );
        break;
    case MAIN_PRIO:
        Local_OutString( "Main," );
        break;
    default:
        Local_OutString( "User," );

    }
    Local_OutString( "#" );
    Local_OutHex( task, 1 );
}

/* read without creating a trap */
DWORD SafeRead(DWORD dw)
{
    if ( ( dw < 0x20000200 ) || ( dw >= (DWORD) _heapend ) )
        return 0;

    return *( (PDWORD) dw );
}

void WalkStack(OS_TCB * pTcb, DWORD newpc, DWORD olda6)
{
    int n = 0;
    while ( n < 5 )
    {

        newpc = SafeRead( olda6 + 4 );
        olda6 = SafeRead( olda6 );
        if ( ( olda6 < (DWORD) pTcb->OSTCBStkPtr ) || ( newpc == (DWORD) OSTaskDelete ) )
        {
            Local_OutHex( newpc, 4 );
            Local_OutString( "?" );
            break;
        } else
            Local_OutHex( newpc, 4 );
        Local_OutString( "," );
        n++;
    }
}

void Local_OutTCBState(OS_TCB * pTcb, DWORD pc, DWORD a6)
{

    DWORD * pStack;
    DWORD newpc;
    DWORD olda6;
    Local_OutString( "\r\n" );
    Local_ShowTaskName( pTcb->OSTCBPrio );
    Local_OutString( "|" );

    if ( OSTCBCur->OSTCBPrio == pTcb->OSTCBPrio )
    {
        Local_OutString( "Running   |    |" );
        Local_OutHex( pc, 4 );
        Local_OutString( "," );
        WalkStack( pTcb, pc, a6 );
    } else
    {
        switch ( pTcb->OSTCBStat ) {
        case OS_STAT_RDY:
            if ( pTcb->OSTCBDly == 0 )
                Local_OutString( "Ready     |" );
            else
                Local_OutString( "Timer     |" );
            break;
        case OS_STAT_MBOX:
            Local_OutString( "Mailbox   |" );
            break;
        case OS_STAT_SEM:
            Local_OutString( "Semaphore |" );
            break;
        case OS_STAT_Q:
            Local_OutString( "Queue     |" );
            break;
        case OS_STAT_FIFO:
            Local_OutString( "Fifo      |" );
            break;
        case OS_STAT_CRIT:
            Local_OutString( "Critical  |" );
            break;
        default:
            Local_OutString( "???? =" );
            Local_OutHex( pTcb->OSTCBStat, 1 );
            Local_OutString( "  |" );

            break;
        }

        if ( ( pTcb->OSTCBStat == OS_STAT_RDY ) && ( pTcb->OSTCBDly == 0 ) )
            Local_OutString( "    |" );
        else
        {
            Local_OutHex( pTcb->OSTCBDly, 2 );
            Local_OutString( "|" );
        }

        pStack = (PDWORD) pTcb->OSTCBStkPtr;
        olda6 = pStack[14];
        pStack += 15;
        //Now we walk up the stack....simulate the RTE....
        newpc = pStack[1];
        if ( newpc == (DWORD) &UCOSWAITS_HERE )
        {
            WalkStack( pTcb, newpc, olda6 );
        } else
        {
            Local_OutHex( newpc, 4 );
            Local_OutString( "," );
            WalkStack( pTcb, newpc, olda6 );
        }
    }

}

void Smart_Trap_Holder()
{
    asm(".global smart_trap_asm");
    asm(".extern smart_trap_a7");
    asm(".extern smart_trap_stack");
    asm("smart_trap_asm:");
    asm("move.w #0x2700,%sr ");
    asm("move.l %a7,smart_trap_a7");
    asm("move.l #smart_trap_stack,%a7");
    asm("add.l #1024,%a7");
    asm("lea      -60(%a7),%a7 ");
    asm("movem.l  %d0-%d7/%a0-%a6,(%a7) ");

    /* C part of functions starts here */

    if ( already_in_trap )
        ForceReboot();
    already_in_trap = TRUE;
    while ( 1 )
    {
        BOOL corrupt = FALSE;
        WORD old_sr;
        DWORD pc;
        DWORD a6;

        Local_OutString( "\r\n-------------------Trap information-----------------------------\r\n" );

        /* Step 1 check to see if A7 is reasonable */
        if ( ( smart_trap_a7 < 0x02000000 ) || ( smart_trap_a7 > (DWORD) _heapend ) )
        {
            Local_OutString( "Stack is corrupt A7=" );
            corrupt = TRUE;
        } else
        {
            Local_OutString( "Exception Frame/A7 =" );
        }
        Local_OutHex( smart_trap_a7, 4 );
        Local_OutString( "\r\nTrap Vector        =" );

		cap_trap.a7=smart_trap_a7;

        if ( !corrupt )
        {
            pc = *( ( (PDWORD) smart_trap_a7 ) + 1 );
            DWORD frame = *( (PDWORD) smart_trap_a7 );
            DWORD n = ( frame >> 18 ) & 0xFF;
			cap_trap.vec=n;
			cap_trap.pc=pc;

            switch ( n ) {
            case 2:
                Local_OutString( "Access Error (2)" );
                break;
            case 3:
                Local_OutString( "Address Error (3)" );
                break;
            case 4:
                Local_OutString( "Illegal Instruction (4)" );
                break;
            case 5:
                Local_OutString( "Divide by Zero (5)" );
                break;
            case 8:
                Local_OutString( "Privledge Violation (8)" );
                break;
            case 9:
                Local_OutString( "Trace (9)" );
                break;
            case 10:
                Local_OutString( "Unimplemented line-a opcode (10)" );
                break;
            case 11:
                Local_OutString( "Unimplemented line-f opcode (11)" );
                break;
            case 12:
                Local_OutString( "Debug interupt (12)" );
                break;
            case 14:
                Local_OutString( "Format Error (14)" );
                break;
            case 15:
                Local_OutString( "Unititialized Interrupt (15)" );
                break;
            default:
                Local_OutString( "???? (Hex):0x" );
                Local_OutHex( n, 1 );
                break;
            }

            Local_OutString( "\r\nFormat             =" );
            Local_OutHex( ( frame >> 26 ) & 0x0F, 1 );
            old_sr = frame & 0xFFFF;
            Local_OutString( "\r\nStatus register SR =" );
            Local_OutHex( old_sr, 2 );
            frame = frame >> 16;
            frame = ( frame & 0x3 ) | ( ( frame >> 8 ) & 0x0C );
            Local_OutString( "\r\nFault Status       =" );
            Local_OutHex( frame & 0x0F, 1 );
            Local_OutString( "\r\nFaulted PC         =" );
            Local_OutHex( pc, 4 );
            Local_OutString( "\r\n" );
            Local_OutString( "\r\n-------------------Register information-------------------------\r\n" );

            a6 = smart_trap_stack[249 + 6];

            for ( int i = 0; i < 7; i++ )
            {
                LocalOutByte( 'A' );
                LocalOutByte( '0' + i );
                LocalOutByte( '=' );
                Local_OutHex( smart_trap_stack[249 + i], 4 );
                if ( i == 3 )
                    Local_OutString( "\r\n" );
                else
                    LocalOutByte( ' ' );
            }
            Local_OutString( "A7=" );
            Local_OutHex( smart_trap_a7, 4 );
            LocalOutByte( '\r' );
            LocalOutByte( '\n' );

            for ( int i = 0; i < 8; i++ )
            {
                LocalOutByte( 'D' );
                LocalOutByte( '0' + i );
                LocalOutByte( '=' );
                Local_OutHex( smart_trap_stack[241 + i], 4 );
                if ( i == 3 )
                    Local_OutString( "\r\n" );
                else
                    LocalOutByte( ' ' );
            }
            Local_OutString( "\r\nSR=" );
            Local_OutHex( old_sr, 2 );
            Local_OutString( " PC=" );
            Local_OutHex( pc, 4 );

            Local_OutString( "\r\n-------------------RTOS information-----------------------------\r\n" );

            if ( old_sr & 0x0F00 )
                Local_OutString( "SR indicates trap from within ISR or CRITICAL RTOS section\r\n" );
            if ( OSIntNesting > 0 )
            {
                Local_OutString( "OSIntNesting indicates trap from within ISR OSIntNesting=" );
                Local_OutHex( OSIntNesting, 1 );
                Local_OutString( "\r\n" );
            }

            Local_OutString( "The OSTCBCur current task control block = " );
            Local_OutHex( (DWORD) OSTCBCur, 4 );
            int i;
            for ( i = 0; i < OS_MAX_TASKS; i++ )
            {
                if ( OSTCBTbl + i == OSTCBCur )
                    break;
            }
            if ( i == OS_MAX_TASKS )
            {
                Local_OutString( "\r\nThis is not a valid TCB the RTOS has been corrupted\r\n" );
            } else
            {
                Local_OutString( "\r\nThis looks like a valid TCB\r\nThe current running task is: " );
                Local_ShowTaskName( OSTCBCur->OSTCBPrio );
				cap_trap.prio=OSTCBCur->OSTCBPrio;
                Local_OutString( "\r\n" );
            }

            Local_OutString( "-------------------Task information-----------------------------\r\n" );
            Local_OutString( "Task    | State    |Wait| Call Stack" );

            for ( i = 0; i < OS_MAX_TASKS; i++ )
                if ( OSTCBTbl[i].OSTCBPrio )
                {
                    Local_OutTCBState( OSTCBTbl + i, pc, a6 );
                }
            Local_OutString( "\r\n" );

        } else
            Local_OutString( "\r\nStack Frame seems corrupt unable to do RTOS dump\r\n" );

        Local_OutString( "\r\n-------------------End of Trap Diagnostics----------------------\r\n" );

        ForceReboot();
    }

}

void EnableSmartTraps()
{
    for ( int i = 2; i < 32; i++ )
        vector_base.table[i] = (DWORD) &smart_trap_asm;
    for ( int i = 32; i < 256; i++ )
        if ( ( i != 42 ) && ( vector_base.table[i] < 0xFFC04000 ) )
            vector_base.table[i] = (DWORD) &smart_trap_asm;

}


