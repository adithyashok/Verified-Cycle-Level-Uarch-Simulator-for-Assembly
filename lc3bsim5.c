/***************************************************************/
/*                                                             */
/*   LC-3b Simulator, adithya                                  */
/*                                                             */
/*   The University of Texas at Austin                         */
/*                                                             */
/***************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/***************************************************************/
/*                                                             */
/* Files:  ucode        Microprogram file (Encrypted)          */
/*         pagetable    page table in LC-3b machine language   */
/*         isaprogram   LC-3b machine language program file    */
/*                                                             */
/***************************************************************/

/***************************************************************/
/* These are the functions you'll have to write.               */
/***************************************************************/

void eval_micro_sequencer();
void cycle_memory();
void eval_bus_drivers();
void drive_bus();
void latch_datapath_values();

/***************************************************************/
/* A couple of useful definitions.                             */
/***************************************************************/
#define FALSE 0
#define TRUE  1

/***************************************************************/
/* Use this to avoid overflowing 16 bits on the bus.           */
/***************************************************************/
#define Low16bits(x) ((x) & 0xFFFF)

/***************************************************************/
/* Definition of the control store layout.                     */
/***************************************************************/
#define CONTROL_STORE_ROWS 64
#define INITIAL_STATE_NUMBER 18

/***************************************************************/
/* Definition of bit order in control store word.              */
/***************************************************************/
enum CS_BITS {                                                  
    IRD,
    COND1, COND0,
    J5, J4, J3, J2, J1, J0,
    LD_MAR,
    LD_MDR,
    LD_IR,
    LD_BEN,
    LD_REG,
    LD_CC,
    LD_PC,
    GATE_PC,
    GATE_MDR,
    GATE_ALU,
    GATE_MARMUX,
    GATE_SHF,
    PCMUX1, PCMUX0,
    DRMUX,
    SR1MUX,
    ADDR1MUX,
    ADDR2MUX1, ADDR2MUX0,
    MARMUX,
    ALUK1, ALUK0,
    MIO_EN,
    R_W,
    DATA_SIZE,
    LSHF1,
	//l4
	//
    COND2,
    LD_MODE,
    LD_SSP,
    LD_USP,
    LD_VECT,
    LD_IVT,
    GATE_PSR,
    GATE_PC2,
    GATE_SP,
    GATE_VECT,
    MODEMUX,
    SPMUX1, SPMUX0,
    SR1MUX1, DRMUX1,
    
	//l5
	//
    LD_JREG,
    JMUX,
    TRANSLATE,
    LD_VA,
    GATE_VADDR,
    GATE_PADDR,
/* MODIFY: you have to add all your new control signals */
    CONTROL_STORE_BITS
} CS_BITS;

/***************************************************************/
/* Functions to get at the control bits.                       */
/***************************************************************/
int GetIRD(int *x)           { return(x[IRD]); }
int GetCOND(int *x)          { return((x[COND1] << 1) + x[COND0]); }
int GetJ(int *x)             { return((x[J5] << 5) + (x[J4] << 4) +
				      (x[J3] << 3) + (x[J2] << 2) +
				      (x[J1] << 1) + x[J0]); }
int GetLD_MAR(int *x)        { return(x[LD_MAR]); }
int GetLD_MDR(int *x)        { return(x[LD_MDR]); }
int GetLD_IR(int *x)         { return(x[LD_IR]); }
int GetLD_BEN(int *x)        { return(x[LD_BEN]); }
int GetLD_REG(int *x)        { return(x[LD_REG]); }
int GetLD_CC(int *x)         { return(x[LD_CC]); }
int GetLD_PC(int *x)         { return(x[LD_PC]); }
int GetGATE_PC(int *x)       { return(x[GATE_PC]); }
int GetGATE_MDR(int *x)      { return(x[GATE_MDR]); }
int GetGATE_ALU(int *x)      { return(x[GATE_ALU]); }
int GetGATE_MARMUX(int *x)   { return(x[GATE_MARMUX]); }
int GetGATE_SHF(int *x)      { return(x[GATE_SHF]); }
int GetPCMUX(int *x)         { return((x[PCMUX1] << 1) + x[PCMUX0]); }
int GetDRMUX(int *x)         { return(x[DRMUX]); }
int GetSR1MUX(int *x)        { return(x[SR1MUX]); }
int GetADDR1MUX(int *x)      { return(x[ADDR1MUX]); }
int GetADDR2MUX(int *x)      { return((x[ADDR2MUX1] << 1) + x[ADDR2MUX0]); }
int GetMARMUX(int *x)        { return(x[MARMUX]); }
int GetALUK(int *x)          { return((x[ALUK1] << 1) + x[ALUK0]); }
int GetMIO_EN(int *x)        { return(x[MIO_EN]); }
int GetR_W(int *x)           { return(x[R_W]); }
int GetDATA_SIZE(int *x)     { return(x[DATA_SIZE]); } 
int GetLSHF1(int *x)         { return(x[LSHF1]); }
/* MODIFY: you can add more Get functions for your new control signals */

/***************************************************************/
/* The control store rom.                                      */
/***************************************************************/
int CONTROL_STORE[CONTROL_STORE_ROWS][CONTROL_STORE_BITS];

/***************************************************************/
/* Main memory.                                                */
/***************************************************************/
/* MEMORY[A][0] stores the least significant byte of word at word address A
   MEMORY[A][1] stores the most significant byte of word at word address A 
   There are two write enable signals, one for each byte. WE0 is used for 
   the least significant byte of a word. WE1 is used for the most significant 
   byte of a word. */

#define WORDS_IN_MEM    0x2000 /* 32 frames */ 
#define MEM_CYCLES      5
int MEMORY[WORDS_IN_MEM][2];

/***************************************************************/

/***************************************************************/

/***************************************************************/
/* LC-3b State info.                                           */
/***************************************************************/
#define LC_3b_REGS 8

int RUN_BIT;	/* run bit */
int BUS;	/* value of the bus */
int INT; //interrupt for l4
typedef struct System_Latches_Struct{

int PC,		/* program counter */
    MDR,	/* memory data register */
    MAR,	/* memory address register */
    IR,		/* instruction register */
    N,		/* n condition bit */
    Z,		/* z condition bit */
    P,		/* p condition bit */
    BEN;        /* ben register */

int READY;	/* ready bit */
  /* The ready bit is also latched as you dont want the memory system to assert it 
     at a bad point in the cycle*/

int REGS[LC_3b_REGS]; /* register file. */

int MICROINSTRUCTION[CONTROL_STORE_BITS]; /* The microintruction */

int STATE_NUMBER; /* Current State Number - Provided for debugging */ 

/* For l 4 */
int INTV; /* Interrupt vector register */
//int EXCV; /* Exception vector register */
int SSP; /* Initial value of system stack pointer */
int USP; //User Stack Pointer
int MODE; //MODE = 1 for user, 0 for system
int IVT;
int VECT; 
/* l4 MODIFY: You may add system latches that are required by your implementation */


/* For la5 */
int PTBR; /* This is initialized when we load the page table */
int VA;   /* Temporary VA register */
int JREG;
/* l5 MODIFY: you should add here any other registers you need to implement virtual memory */

} System_Latches;

/* Data Structure for Latch */

System_Latches CURRENT_LATCHES, NEXT_LATCHES;

/* For l 5 */
#define PAGE_NUM_BITS 9
#define PTE_PFN_MASK 0x3E00
#define PTE_VALID_MASK 0x0004
#define PAGE_OFFSET_MASK 0x1FF

/***************************************************************/
/* A cycle counter.                                            */
/***************************************************************/
int CYCLE_COUNT;

/***************************************************************/
/*                                                             */
/* Procedure : help                                            */
/*                                                             */
/* Purpose   : Print out a list of commands.                   */
/*                                                             */
/***************************************************************/
void help() {                                                    
    printf("----------------LC-3bSIM Help-------------------------\n");
    printf("go               -  run program to completion       \n");
    printf("run n            -  execute program for n cycles    \n");
    printf("mdump low high   -  dump memory from low to high    \n");
    printf("rdump            -  dump the register & bus values  \n");
    printf("?                -  display this help menu          \n");
    printf("quit             -  exit the program                \n\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : cycle                                           */
/*                                                             */
/* Purpose   : Execute a cycle                                 */
/*                                                             */
/***************************************************************/
void cycle() {                                                
 
  //l4
  if(CYCLE_COUNT == 300){
	  INT  = 1;
	  NEXT_LATCHES.INTV = 0x01;
  }


  eval_micro_sequencer();   
  cycle_memory();
  eval_bus_drivers();
  drive_bus();
  latch_datapath_values();

  CURRENT_LATCHES = NEXT_LATCHES;

  CYCLE_COUNT++;
}

/***************************************************************/
/*                                                             */
/* Procedure : run n                                           */
/*                                                             */
/* Purpose   : Simulate the LC-3b for n cycles.                 */
/*                                                             */
/***************************************************************/
void run(int num_cycles) {                                      
    int i;

    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating for %d cycles...\n\n", num_cycles);
    for (i = 0; i < num_cycles; i++) {
	if (CURRENT_LATCHES.PC == 0x0000) {
	    RUN_BIT = FALSE;
	    printf("Simulator halted\n\n");
	    break;
	}
	cycle();
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : go                                              */
/*                                                             */
/* Purpose   : Simulate the LC-3b until HALTed.                 */
/*                                                             */
/***************************************************************/
void go() {                                                     
    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating...\n\n");
    while (CURRENT_LATCHES.PC != 0x0000)
	cycle();
    RUN_BIT = FALSE;
    printf("Simulator halted\n\n");
}

/***************************************************************/ 
/*                                                             */
/* Procedure : mdump                                           */
/*                                                             */
/* Purpose   : Dump a word-aligned region of memory to the     */
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void mdump(FILE * dumpsim_file, int start, int stop) {          
    int address; /* this is a byte address */

    printf("\nMemory content [0x%0.4x..0x%0.4x] :\n", start, stop);
    printf("-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	printf("  0x%0.4x (%d) : 0x%0.2x%0.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    printf("\n");

    /* dump the memory contents into the dumpsim file */
    fprintf(dumpsim_file, "\nMemory content [0x%0.4x..0x%0.4x] :\n", start, stop);
    fprintf(dumpsim_file, "-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	fprintf(dumpsim_file, " 0x%0.4x (%d) : 0x%0.2x%0.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : rdump                                           */
/*                                                             */
/* Purpose   : Dump current register and bus values to the     */   
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void rdump(FILE * dumpsim_file) {                               
    int k; 

    printf("\nCurrent register/bus values :\n");
    printf("-------------------------------------\n");
    printf("Cycle Count  : %d\n", CYCLE_COUNT);
    printf("PC           : 0x%0.4x\n", CURRENT_LATCHES.PC);
    printf("IR           : 0x%0.4x\n", CURRENT_LATCHES.IR);
    printf("STATE_NUMBER : 0x%0.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    printf("BUS          : 0x%0.4x\n", BUS);
    printf("MDR          : 0x%0.4x\n", CURRENT_LATCHES.MDR);
    printf("MAR          : 0x%0.4x\n", CURRENT_LATCHES.MAR);
    printf("CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    printf("Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	printf("%d: 0x%0.4x\n", k, CURRENT_LATCHES.REGS[k]);
    printf("\n");

    /* dump the state information into the dumpsim file */
    fprintf(dumpsim_file, "\nCurrent register/bus values :\n");
    fprintf(dumpsim_file, "-------------------------------------\n");
    fprintf(dumpsim_file, "Cycle Count  : %d\n", CYCLE_COUNT);
    fprintf(dumpsim_file, "PC           : 0x%0.4x\n", CURRENT_LATCHES.PC);
    fprintf(dumpsim_file, "IR           : 0x%0.4x\n", CURRENT_LATCHES.IR);
    fprintf(dumpsim_file, "STATE_NUMBER : 0x%0.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    fprintf(dumpsim_file, "BUS          : 0x%0.4x\n", BUS);
    fprintf(dumpsim_file, "MDR          : 0x%0.4x\n", CURRENT_LATCHES.MDR);
    fprintf(dumpsim_file, "MAR          : 0x%0.4x\n", CURRENT_LATCHES.MAR);
    fprintf(dumpsim_file, "CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    fprintf(dumpsim_file, "Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	fprintf(dumpsim_file, "%d: 0x%0.4x\n", k, CURRENT_LATCHES.REGS[k]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : get_command                                     */
/*                                                             */
/* Purpose   : Read a command from standard input.             */  
/*                                                             */
/***************************************************************/
void get_command(FILE * dumpsim_file) {                         
    char buffer[20];
    int start, stop, cycles;

    printf("LC-3b-SIM> ");

    scanf("%s", buffer);
    printf("\n");

    switch(buffer[0]) {
    case 'G':
    case 'g':
	go();
	break;

    case 'M':
    case 'm':
	scanf("%i %i", &start, &stop);
	mdump(dumpsim_file, start, stop);
	break;

    case '?':
	help();
	break;
    case 'Q':
    case 'q':
	printf("Bye.\n");
	exit(0);

    case 'R':
    case 'r':
	if (buffer[1] == 'd' || buffer[1] == 'D')
	    rdump(dumpsim_file);
	else {
	    scanf("%d", &cycles);
	    run(cycles);
	}
	break;

    default:
	printf("Invalid Command\n");
	break;
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : init_control_store                              */
/*                                                             */
/* Purpose   : Load microprogram into control store ROM        */ 
/*                                                             */
/***************************************************************/
void init_control_store(char *ucode_filename) {                 
    FILE *ucode;
    int i, j, index;
    char line[200];

    printf("Loading Control Store from file: %s\n", ucode_filename);

    /* Open the micro-code file. */
    if ((ucode = fopen(ucode_filename, "r")) == NULL) {
	printf("Error: Can't open micro-code file %s\n", ucode_filename);
	exit(-1);
    }

    /* Read a line for each row in the control store. */
    for(i = 0; i < CONTROL_STORE_ROWS; i++) {
	if (fscanf(ucode, "%[^\n]\n", line) == EOF) {
	    printf("Error: Too few lines (%d) in micro-code file: %s\n",
		   i, ucode_filename);
	    exit(-1);
	}

	/* Put in bits one at a time. */
	index = 0;

	for (j = 0; j < CONTROL_STORE_BITS; j++) {
	    /* Needs to find enough bits in line. */
	    if (line[index] == '\0') {
		printf("Error: Too few control bits in micro-code file: %s\nLine: %d\n",
		       ucode_filename, i);
		exit(-1);
	    }
	    if (line[index] != '0' && line[index] != '1') {
		printf("Error: Unknown value in micro-code file: %s\nLine: %d, Bit: %d\n",
		       ucode_filename, i, j);
		exit(-1);
	    }

	    /* Set the bit in the Control Store. */
	    CONTROL_STORE[i][j] = (line[index] == '0') ? 0:1;
	    index++;
	}

	/* Warn about extra bits in line. */
	if (line[index] != '\0')
	    printf("Warning: Extra bit(s) in control store file %s. Line: %d\n",
		   ucode_filename, i);
    }
    printf("\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : init_memory                                     */
/*                                                             */
/* Purpose   : Zero out the memory array                       */
/*                                                             */
/***************************************************************/
void init_memory() {                                           
    int i;

    for (i=0; i < WORDS_IN_MEM; i++) {
	MEMORY[i][0] = 0;
	MEMORY[i][1] = 0;
    }
}

/**************************************************************/
/*                                                            */
/* Procedure : load_program                                   */
/*                                                            */
/* Purpose   : Load program and service routines into mem.    */
/*                                                            */
/**************************************************************/
void load_program(char *program_filename, int is_virtual_base) {                   
    FILE * prog;
    int ii, word, program_base, pte, virtual_pc;

    /* Open program file. */
    prog = fopen(program_filename, "r");
    if (prog == NULL) {
	printf("Error: Can't open program file %s\n", program_filename);
	exit(-1);
    }

    /* Read in the program. */
    if (fscanf(prog, "%x\n", &word) != EOF)
	program_base = word >> 1;
    else {
	printf("Error: Program file is empty\n");
	exit(-1);
    }

    if (is_virtual_base) {
      if (CURRENT_LATCHES.PTBR == 0) {
	printf("Error: Page table base not loaded %s\n", program_filename);
	exit(-1);
      }

      /* convert virtual_base to physical_base */
      virtual_pc = program_base << 1;
      pte = (MEMORY[(CURRENT_LATCHES.PTBR + (((program_base << 1) >> PAGE_NUM_BITS) << 1)) >> 1][1] << 8) | 
	     MEMORY[(CURRENT_LATCHES.PTBR + (((program_base << 1) >> PAGE_NUM_BITS) << 1)) >> 1][0];

      printf("virtual base of program: %04x\npte: %04x\n", program_base << 1, pte);
		if ((pte & PTE_VALID_MASK) == PTE_VALID_MASK) {
	      program_base = (pte & PTE_PFN_MASK) | ((program_base << 1) & PAGE_OFFSET_MASK);
   	   printf("physical base of program: %x\n\n", program_base);
	      program_base = program_base >> 1; 
		} else {
   	   printf("attempting to load a program into an invalid (non-resident) page\n\n");
			exit(-1);
		}
    }
    else {
      /* is page table */
     CURRENT_LATCHES.PTBR = program_base << 1;
    }

    ii = 0;
    while (fscanf(prog, "%x\n", &word) != EOF) {
	/* Make sure it fits. */
	if (program_base + ii >= WORDS_IN_MEM) {
	    printf("Error: Program file %s is too long to fit in memory. %x\n",
		   program_filename, ii);
	    exit(-1);
	}

	/* Write the word to memory array. */
	MEMORY[program_base + ii][0] = word & 0x00FF;
	MEMORY[program_base + ii][1] = (word >> 8) & 0x00FF;;
	ii++;
    }

    if (CURRENT_LATCHES.PC == 0 && is_virtual_base) 
      CURRENT_LATCHES.PC = virtual_pc;

    printf("Read %d words from program into memory.\n\n", ii);
}

/***************************************************************/
/*                                                             */
/* Procedure : initialize                                      */
/*                                                             */
/* Purpose   : Load microprogram and machine language program  */ 
/*             and set up initial state of the machine         */
/*                                                             */
/***************************************************************/
void initialize(char *argv[], int num_prog_files) { 
    int i;
    init_control_store(argv[1]);

    init_memory();
    load_program(argv[2],0);
    for ( i = 0; i < num_prog_files; i++ ) {
	load_program(argv[i + 3],1);
    }
    CURRENT_LATCHES.Z = 1;
    CURRENT_LATCHES.MODE = 1;
    CURRENT_LATCHES.STATE_NUMBER = INITIAL_STATE_NUMBER;
    memcpy(CURRENT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[INITIAL_STATE_NUMBER], sizeof(int)*CONTROL_STORE_BITS);
    CURRENT_LATCHES.SSP = 0x3000; /* Initial value of system stack pointer */
    CURRENT_LATCHES.USP = 0xFE00; //l4 USP set
/* MODIFY: you can add more initialization code HERE */

    NEXT_LATCHES = CURRENT_LATCHES;

    RUN_BIT = TRUE;
}

/***************************************************************/
/*                                                             */
/* Procedure : main                                            */
/*                                                             */
/***************************************************************/
int main(int argc, char *argv[]) {                              
    FILE * dumpsim_file;

    /* Error Checking */
    if (argc < 4) {
	printf("Error: usage: %s <micro_code_file> <page table file> <program_file_1> <program_file_2> ...\n",
	       argv[0]);
	exit(1);
    }

    printf("LC-3b Simulator\n\n");

    initialize(argv, argc - 3);

    if ( (dumpsim_file = fopen( "dumpsim", "w" )) == NULL ) {
	printf("Error: Can't open dumpsim file\n");
	exit(-1);
    }

    while (1)
	get_command(dumpsim_file);

}

/***************************************************************/
/* Do not modify the above code, except for the places indicated 
   with a "MODIFY:" comment.
   You are allowed to use the following global variables in your
   code. These are defined above.

   CONTROL_STORE
   MEMORY
   BUS

   CURRENT_LATCHES
   NEXT_LATCHES

   You may define your own local/global variables and functions.
   You may use the functions to get at the control bits defined
   above.

   Begin your code here 	  			       */
/***************************************************************/



int sext(int val, int start, int end){

	if (val>>(start-1) == 1) {

		int i = end - start;
		int a = 1 << start;

		while (i > 0) {
			val += a;
			a = a << 1;
			i--;
		}
	}
	return val;

}

//questions for TA: When exactly do I need to use the Low16bits functions.
//how do do 0b instead of 0x
//

int UA, PROT = 0;//l4
int PF = 0; //l5
int JREG_IN;
void eval_micro_sequencer() {

	/* 
	 * Evaluate the address of the next state according to the 
i	 * micro sequencer logic. Latch the next microinstruction.
	 */
//	printf("PTBR: %x\n", CURRENT_LATCHES.PTBR);
//	printf("VA: %x\n", CURRENT_LATCHES.VA);
//	printf("JREG: %x\n", CURRENT_LATCHES.JREG);
	int *micro = CURRENT_LATCHES.MICROINSTRUCTION;

	int ird = micro[IRD];

	int j0 = micro[J0];
	int j1 = micro[J1];
	int j2 = micro[J2];
	int j3 = micro[J3];
	int j4 = micro[J4];
	int j5 = micro[J5];
	
	int j = GetJ(micro);
	
//	printf("j: %d\n", j);	
	
	int ben  = CURRENT_LATCHES.BEN;
	int ready = CURRENT_LATCHES.READY;
	int ir11 = (CURRENT_LATCHES.IR>>11) - (CURRENT_LATCHES.IR>>12<<1);

	int cond0 = micro[COND0];
	int cond1 = micro[COND1];
	int cond2 = micro[COND2];//l4
	
	PROT = 0;
			
	int PTE2 = (CURRENT_LATCHES.MDR >> 2) - (CURRENT_LATCHES.MDR >> 3 << 1);//l5
	int PTE3 = (CURRENT_LATCHES.MDR >> 3) - (CURRENT_LATCHES.MDR >> 4 << 1);//l5
	
	if((!PTE3) && CURRENT_LATCHES.MODE == 1 && (CURRENT_LATCHES.IR >> 12) != 15) PROT = 1;

	PF = 0;//l5
	if(!PTE2) PF = 1;

//	printf("\nPROT: %x", PROT);	
//	printf("\nUA: %x", UA);
//	printf("\nLD: %d", micro[LD_MAR]);
//	printf("\nMODE: %d\n", CURRENT_LATCHES.MODE);

//	if(CURRENT_LATCHES.STATE_NUMBER >=41)printf("MODEMUX: %d\n", micro[MODEMUX]);
	
	int ex = 0;
	int sn = CURRENT_LATCHES.STATE_NUMBER;

	if((CURRENT_LATCHES.IR >>13) == 5){
		
		PROT = 0;
		PF = 0;
		UA = 0;
	}
	if(sn == 6 || sn == 7 || sn == 2 || sn == 3)UA = micro[DATA_SIZE]; //l5
	 
	if(sn == 55){
		 UA = (UA && (CURRENT_LATCHES.VA & 0x0001));//l5
		 ex = PROT | PF | (UA && (CURRENT_LATCHES.VA & 0x0001));//l5
	}
		
	if(ex) j = 10;		
	
	else if(ird) j = CURRENT_LATCHES.IR >> 12;// & 0b001111
	
	else{
	
		j0 = (j0 | (ir11 & cond0 & cond1)) & 0x0001;
		j1 = (j1 | (ready & cond0 & (!cond1))) & 0x0001;
		j2 = (j2 | (ben & (!cond0) & cond1)) & 0x0001 ;
		j3 = (j3 | (INT & cond2)) & 0x0001;//l4
			
		j = j0 + (j1 << 1) + (j2 << 2) + (j3 << 3) + (j4 << 4) + (j5 << 5);

	}

//	printf("j: %d\n", j);

	JREG_IN = j;
	
	if(micro[JMUX]){
		//l5
		j = CURRENT_LATCHES.JREG;
	}

	if(micro[TRANSLATE]) j = 52;

	if(CURRENT_LATCHES.STATE_NUMBER == 51) 
	{
		INT = 0;
		PROT = 0;
		UA = 0;
		PF = 0;
	}
		
	memcpy(NEXT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[j], sizeof(int)*CONTROL_STORE_BITS);
	NEXT_LATCHES.STATE_NUMBER = j;

}

int CYCLE = 1;
int MDR_SET;

void cycle_memory() {

	/* 
	 * This function emulates memory and the WE logic. 
	 * Keep track of which cycle of MEMEN we are dealing with.  

	 * If fourth, we need to latch Ready bit at the end of 
	 * cycle to prepare microsequencer for the fifth cycle.  

*/
	int *micro = CURRENT_LATCHES.MICROINSTRUCTION;
	
	if (micro[MIO_EN])
	{
		if(CYCLE == 4)
		{
			NEXT_LATCHES.READY = 1;
			CYCLE++;
		}

		else if(CYCLE == 5)
		{

			//DO THE READ AND THE WRITE HERE
			if(micro[R_W] == 0){
			
			
				MDR_SET = Low16bits( (MEMORY[CURRENT_LATCHES.MAR/2][1] << 8) + MEMORY[CURRENT_LATCHES.MAR/2][0]);
			
			}

			else{
				//WRITE
				if(micro[DATA_SIZE])
				{//word

					MEMORY[CURRENT_LATCHES.MAR/2][1] = Low16bits( CURRENT_LATCHES.MDR >> 8);
					MEMORY[CURRENT_LATCHES.MAR/2][0] = Low16bits( CURRENT_LATCHES.MDR & 0x00FF);					

				}		
				
				else{//byte
					
					MEMORY[CURRENT_LATCHES.MAR/2][Low16bits(CURRENT_LATCHES.MAR & 0x0001)] = Low16bits(CURRENT_LATCHES.MDR & 0x00FF);	

				}
			}
			CURRENT_LATCHES.MDR = MDR_SET;
			CYCLE = 1;
			NEXT_LATCHES.READY = 0;			
		}

		else{
			CYCLE++;
		}		
	}

	else CYCLE = 1;	
}

int MARMUX_OUT, PC_OUT, ALU_OUT, SHF_OUT, MDR_OUT;

int ADDER; //the address adder

int SPMUX_OUT, VECT_OUT, PC2_OUT, PSR, SR1MUX_OUT; //l4

int VADDR_OUT, PADDR_OUT; //l5

void eval_bus_drivers() {

	/* 
	 * Datapath routine emulating operations before driving the bus.
	 * Evaluate the input of tristate drivers 
	 *             Gate_MARMUX,
	 *		 Gate_PC,
	 *		 Gate_ALU,
	 *		 Gate_SHF,
	 *		 Gate_MDR.
	 */    
	
	int *micro = CURRENT_LATCHES.MICROINSTRUCTION;

	int ir = CURRENT_LATCHES.IR;

	MARMUX_OUT = 0;
	PC_OUT = CURRENT_LATCHES.PC;
	ALU_OUT = 0;
	SHF_OUT = 0;
	MDR_OUT = 0;
	
	//ALU stuff
	//
	
	int SR1, SR2 = 0;
	
	if(micro[SR1MUX1]) SR1 = 6;
	else{
		if(micro[SR1MUX]) SR1 = ( ir>>6) - (ir>>9<<3);
		else SR1 = (ir>>9) - (ir>>12<<3);
	}
	
	SR2 = (ir) - (ir>>3<<3);
	
	int imm5 = (ir>>5) - (ir>>6<<1);
	
	int A, B;

	if (imm5 == 0) {
		B = CURRENT_LATCHES.REGS[SR2];
	}

	else if (imm5 == 1) {
	
		B = ir - (ir>>5<<5);
		B = sext(B, 5, 16);
	
	}
	
	A = CURRENT_LATCHES.REGS[SR1];
	
	switch(GetALUK(micro)){
		
		case 0:{
			ALU_OUT = Low16bits(A + B) ;
			break;
			}
		case 1:
			ALU_OUT =  A & B ;
			break;

		case 2:
			ALU_OUT =  A ^ B;
			break;

		case 3:
			ALU_OUT =  A ;
			break;

		default: break;

	}

	//SHF_OUT
	

	int SHFbit5 = ( ir >> 5 ) - ( ir >> 6 << 1);
	int SHFbit4 = ( ir >> 4 ) - ( ir >> 5 << 1);

	int SHFamount = ir - (ir >> 4 << 4);

	if(SHFbit4 == 0){
		SHF_OUT = Low16bits(CURRENT_LATCHES.REGS[SR1] << SHFamount);
	}
	
	else if (SHFbit5 == 0){
		
		SHF_OUT = (unsigned) CURRENT_LATCHES.REGS[SR1] >> SHFamount;

	}

	else{
		
		if((CURRENT_LATCHES.REGS[SR1] >> 15) == 0){

			SHF_OUT = CURRENT_LATCHES.REGS[SR1] >> SHFamount;

		}

		else{
			int temp = CURRENT_LATCHES.REGS[SR1];
			int tempAmount = SHFamount;

			while(tempAmount --> 0){
				temp  = (temp >> 1) + 0x8000;
			}

			SHF_OUT = temp;

		}
	}

	//MDR_OUT
	//does this look right?
	if(micro[DATA_SIZE]){
	
		MDR_OUT = CURRENT_LATCHES.MDR;

	}

	else{
		if(Low16bits(CURRENT_LATCHES.MAR & 0x0001) == 0) MDR_OUT = (CURRENT_LATCHES.MDR - (CURRENT_LATCHES.MDR >> 8 << 8));

		else MDR_OUT = CURRENT_LATCHES.MDR >> 8;

		MDR_OUT = sext(MDR_OUT, 8, 16);
	}

	//MARMUX
	//
	int addr2mux, addr1mux; 
	
	switch(GetADDR2MUX(micro)){

		case 0: addr2mux = 0;
			break;

		case 1: addr2mux = sext(((ir) - (ir >> 6 << 6 )), 6, 16);
			break;

		case 2: addr2mux = sext(((ir) - (ir >> 9 << 9)), 9, 16);
			break;

		case 3: addr2mux = sext(((ir) - (ir >> 11 << 11)), 11, 16);
			break;

		default: break;
	}


	if(micro[ADDR1MUX]) addr1mux = CURRENT_LATCHES.REGS[SR1];

	else addr1mux = CURRENT_LATCHES.PC;

	if(micro[LSHF1]) addr2mux = addr2mux << 1;

	ADDER = addr1mux + addr2mux;
//	printf("\nAdder nonPC: %x\n", ADDER);


	if(micro[MARMUX]){

		MARMUX_OUT = addr1mux + addr2mux;

	}

	else{

		MARMUX_OUT = ( ir - (ir >> 8 << 8) ) << 1;
		
	}
	
	//l4
	
	VECT_OUT = CURRENT_LATCHES.VECT;		
			
	switch((micro[SPMUX1] << 1) + micro[SPMUX0]){
		
		case 0: SPMUX_OUT = Low16bits(CURRENT_LATCHES.SSP);
			break;
	
		case 1: SPMUX_OUT = Low16bits(CURRENT_LATCHES.USP);
			break;

		case 2: SPMUX_OUT = Low16bits(CURRENT_LATCHES.REGS[6] - 2);
			break;

		case 3: SPMUX_OUT = Low16bits(CURRENT_LATCHES.REGS[6] + 2);
			break;
	}
	
//	if(CURRENT_LATCHES.STATE_NUMBER == 43) printf("\nSSP: %x\n", CURRENT_LATCHES.SSP);
	PC2_OUT = CURRENT_LATCHES.PC - 2;

	PSR = (CURRENT_LATCHES.MODE << 15) + (CURRENT_LATCHES.N << 2) + (CURRENT_LATCHES.Z << 1) + (CURRENT_LATCHES.P);		
	
	SR1MUX_OUT = CURRENT_LATCHES.REGS[SR1];

	//l5
	//VA[15:9] = page number VA[8:0] = Offset
	//PTE[13:9] = PFN
	//
	int pagenumber = (Low16bits(CURRENT_LATCHES.VA) >> 9);
	//printf("VA: %x\n", (CURRENT_LATCHES.VA>>9));
	int offset = CURRENT_LATCHES.VA - (pagenumber << 9);
	int PFN = (CURRENT_LATCHES.MDR >> 9) - (CURRENT_LATCHES.MDR >> 14 << 5);
	
	VADDR_OUT = CURRENT_LATCHES.PTBR + (pagenumber << 1);
	PADDR_OUT = (PFN << 9) + offset;
		
}


void drive_bus() {

	/* 
	 * Datapath routine for driving the bus from one of the 5 possible 
	 * tristate drivers. 
	 */       
	//should I use low16bits herie
	//
	int *micro = CURRENT_LATCHES.MICROINSTRUCTION;

	if (micro[GATE_PC]) BUS = PC_OUT;
	else if(micro[GATE_ALU]) BUS = ALU_OUT;
	else if(micro[GATE_MARMUX]) BUS = MARMUX_OUT;
	else if(micro[GATE_MDR]) BUS = MDR_OUT;
	else if(micro[GATE_SHF]) BUS = SHF_OUT;
	else if(micro[GATE_SP]) BUS = SPMUX_OUT;//l4
	else if(micro[GATE_VECT]) BUS = VECT_OUT;
	else if(micro[GATE_PC2]) BUS = PC2_OUT;
	else if(micro[GATE_PSR]) BUS = PSR;
	else if(micro[GATE_VADDR]) BUS = VADDR_OUT;//l5
	else if(micro[GATE_PADDR]) BUS = PADDR_OUT;
	else BUS = 0;

	BUS = Low16bits(BUS);	
}

void latch_datapath_values() {

	/* 
	 * Datapath routine for computing all functions that need to latch
	 * values in the data path at the end of this cycle.  Some values
	 * require sourcing the bus; therefore, this routine has to come 
	 * after drive_bus.
	 */       
	int *micro = CURRENT_LATCHES.MICROINSTRUCTION;

	int ir  = CURRENT_LATCHES.IR;
	
	if(micro[LD_IR]) NEXT_LATCHES.IR = BUS;

	if(micro[LD_MAR]) NEXT_LATCHES.MAR = BUS;

	if(micro[LD_CC]){
		
		if(micro[MODEMUX]){ //l4
			
			NEXT_LATCHES.N = (BUS >> 2) - (BUS >> 3 << 1) ;
			NEXT_LATCHES.Z = (BUS >> 1) - (BUS >> 2 << 1);
			NEXT_LATCHES.P = BUS - (BUS >> 1 << 1);

		}

		else{
			NEXT_LATCHES.N = 0;
			NEXT_LATCHES.Z = 0;
			NEXT_LATCHES.P = 0;

			if(BUS == 0) NEXT_LATCHES.Z = 1;

			else if( BUS > 32767) NEXT_LATCHES.N = 1;

			else NEXT_LATCHES.P  = 1;
		}
	}

	if(micro[LD_BEN]){

		int n = (ir>>11) - (ir>>11<<1);
		int z = (ir>>10) - (ir>>10<<1);
		int p = (ir>>9) - (ir>>9<<1);

		if (n & CURRENT_LATCHES.N || z & CURRENT_LATCHES.Z || p & CURRENT_LATCHES.P)
			NEXT_LATCHES.BEN = 1;
		else
			NEXT_LATCHES.BEN = 0;
	}

	if(micro[LD_PC]){
		
		switch((GetPCMUX(micro))){

				case 0:
					NEXT_LATCHES.PC = Low16bits(CURRENT_LATCHES.PC + 2);
					break;
				case 1:
					NEXT_LATCHES.PC = Low16bits(BUS);
					break;
				case 2: 
					NEXT_LATCHES.PC = Low16bits(ADDER);
//					printf("\nAdder : %x\n", ADDER);
					break;

				default: break;	

		}

	}
	
	int DR;

	if(micro[LD_REG]){
		if(micro[DRMUX1]) DR  = 6;
		
		else{		
			if(micro[DRMUX] == 0)	DR = (ir >> 9) - (ir >> 12 << 3); 

			else	DR = 7;
		}
		
		NEXT_LATCHES.REGS[DR] = BUS;
			
	}

	if(micro[LD_MDR]){
		
		if(micro[MIO_EN]) NEXT_LATCHES.MDR = MDR_SET;
		

		else{
			if(micro[DATA_SIZE]){
			//word
				NEXT_LATCHES.MDR = Low16bits(BUS);

			}

			else {

				NEXT_LATCHES.MDR = sext(Low16bits((BUS - (BUS >> 8 << 8) )), 8, 16);
			
			}
		
		}

		//l5

		if(CURRENT_LATCHES.STATE_NUMBER == 55){
			
			NEXT_LATCHES.MDR = (CURRENT_LATCHES.MDR >> 1 << 1) + 1;
			if(CURRENT_LATCHES.JREG != 33) NEXT_LATCHES.MDR = NEXT_LATCHES.MDR | (((ir >> 12) - (ir >> 13 << 1)) << 1);
		}	
	}

	//l4
	//
	
	if(micro[LD_MODE]){
		if(micro[MODEMUX]) NEXT_LATCHES.MODE = (BUS >> 15) & 0x0001;
		else NEXT_LATCHES.MODE = 0;
		
	}

	if(micro[LD_USP]) NEXT_LATCHES.USP = SR1MUX_OUT;

	if(micro[LD_SSP]) NEXT_LATCHES.SSP = SR1MUX_OUT;

	int invalid_OPC = (ir >> 13) == 5;

	int OPC = ~UA & ~PROT & ~PF & invalid_OPC; //l5
	
	int ENCODER_OUT;
	//changed vector table values and comparator inputs for l5
	if(INT) ENCODER_OUT = 3;
	else if(PROT) ENCODER_OUT = 2;
	else if(PF) ENCODER_OUT = 1;
	else ENCODER_OUT = 0;
	
	int IVT_IN;

	switch(ENCODER_OUT){

		case 0: IVT_IN = 0x05;
			break;
	
		case 1: IVT_IN = 0x02;
			break;

		case 2: IVT_IN = 0x04;
			break;
			
		case 3: IVT_IN = CURRENT_LATCHES.INTV;			
			break;

	}
	//additional mux for l5
	if(UA)IVT_IN = 0x03;

	if(micro[LD_IVT]) NEXT_LATCHES.IVT = IVT_IN;

	int VECT_IN = 0x0200 + (CURRENT_LATCHES.IVT << 1);
	
	if(micro[LD_VECT]) NEXT_LATCHES.VECT = VECT_IN;

//l5
//
	
	if(micro[LD_JREG] == 0) NEXT_LATCHES.JREG = JREG_IN;
	if(micro[LD_VA]) NEXT_LATCHES.VA = BUS;
	
/*	if(CURRENT_LATCHES.STATE_NUMBER > 41){

	printf("\nINT: %d", INT);
	printf("\nVECT:  %x", CURRENT_LATCHES.VECT);
	printf("\nIVT: %x", CURRENT_LATCHES.IVT);
	printf("\nINTV: %x", CURRENT_LATCHES.INTV);
	}*/

}

