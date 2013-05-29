
typedef struct 
{
 DWORD vec;
 DWORD pc;
 DWORD a7;
 DWORD prio;
}Captured_Trap;

extern volatile Captured_Trap cap_trap;



