
typedef struct 
{
 DWORD vec;
 DWORD pc;
 DWORD a7;
 DWORD prio;
 DWORD sr;
 DWORD frame;
 DWORD next;
 DWORD lab;
}Captured_Trap;

extern volatile Captured_Trap cap_trap;



