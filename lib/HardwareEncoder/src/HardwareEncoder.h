/*

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in the
Software without restriction, including without limitation the rights to use, copy,
modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject to the
following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. */

/* Generates 32 bit position count from 16 bit Quadrature decoder counter.
 *   To do this the total accumulated position is updated every time 
 *     the counter overflows/underflows.  The hardware generates an 
 *     interrupt at this overflow(TOF).  The problem is that the position
 *     can reverse around this area.  And we can't just use the typical 
 *     stategy as with interrupts of disabling the input temporairly.
 *   To solve this, the position is sampled several times between TOF to 
 *     determine direction and to know when we are well away from TOF and 
 *     can do some updating.  I use only one position compare resource,
 *     it would have been more straightforward to use two but I want to 
 *     save one for a precise latch position with an external switch. 
 *
 *  The routines we have are:
 *    setup() - constructor order of execution is not guaranteed, to this
 *              is called to set everyting up at the appropriate time.
 *    start() - starts the counter at zero
 *    ftm_isr() - Interrupt service routine.  Service the TOF and Compare
 *              interrupts from the counter.  Increment the basePosn if 
 *              we overflow in the same direction as before.  Use the
 *              Compare interrupts to keep track of direction.
 *    zeroFTM() - zeroes the counter
 *    calcPosn() - gets the current position.  Needs to add the current 
 *              counter position to the basePosn.  Accurate to within 
 *              interrupt latency and processing delay. 
 *
 * HARDWARE DETAILS
 *
 *    ENCXA	 3	PTA12	 28	Input	    FTM1    7
 *    ENCXB	 4	PTA13	 29	Input	    FTM1    7
 *    ENCYA	32	PTB18	 41	Input	    FTM2    6
 *    ENCYB	25	PTB19	 42	Input	    FTM2    6
 *
 *Pin Control Registers PORTx_PCRn
 * Bit 10-8 MUX Pin Mux Control 1-7 alternative
 *   Alternative 1 is GPIO
 * Bit 4 PFE Passive Filter Enable 1 to enable
 * Bit 1 PE Pull Enable 1 to enable
 * Bit 0 PS Pull Select 0 is pulldown 1 is pullup
 *
 * The following registers are set in pins_teensy.c
 *        and need to be reset.
 *     FTM1_SC
 *     FTM1_CNT
 *     FTM1_MOD
 *     FTM1_C0SC
 *     FTM1_C1SC
 *
 *   FTM1_CNT Counter value
 *      Bit 15-0 is counter value
 *      Writing any value updates counter with CNTIN 
 *   FTM1_MOD Modulo (Max value)
 *	Bit 15-0 is counter value - set to 0xFFFF
 *	Write to CNT first
 *   FTM1_MODE Features Mode Selection
 *      Bit0 FTMEN FTM Enable - [WPDIS must be 1 to write]
 *      Bit2 WPDIS Write Protect to Disable
 *      Set WPDIS, then set FTMEN - must be set to access FTM regs
 *   FTM1_FMS Fault Mode Status
 *       Bit 6 WPEN Write Protect Enable
 *         Write 1 to clear WPDIS
 *   FTM1_FILTER
 *	Filter out pulses shorter than CHxFVALx 4 system clocks
 *	Bit 7-4 CH1FVAL for PHB
 *	Bit 3-0 CH0FVAL for PHA
 *   FTM1_QDCTRL Quadrature Decoder Control and Status
 *	Bit 7 PHAFLTREN Phase A Filter Enable
 *	Bit 6 PHBFLTREN Phase B Filter Enable
 *	Bit 5 PHAPOL Phase A Polarity
 *	Bit 4 PHBPOL Phase B Polarity
 *	Bit 3 QUADMODE Quadrature Decoder Mode
 *	  0 for Quadrature
 *	Bit 2 QUADIR Counting Direction
 *	Bit 1 TOFDIR Timer Overflow Dir
 *	  0 was set on bottom of counting
 *	  1 was set on top of counting
 *	Bit 0 QUADEN Quadrature Mode Enable
 *	  1 is quadrature mode enabled [WPDIS to write]
 *   FTM1_SC FTM1 Status and Control
 *     Bit 7 TOF Timer Overflow Flag
 *     Bit 6 TOIE Timer Overflow Interrupt Enable
 *     Bit 5 CPWMS Center Aligned PWM Select
 *       Resets to 0 (Write when WPDIS is 1)
 *     Rest of bits are 0 (Write when WPDIS is 1)
 *   FTM1_C0SC  FTM1 Channel 0 Status and Control
 *     Set WPDIS before writing control bits
 *     Bit 7 CHF Channel Flag
 *       Channel event occured
 *       Read and write 0 to clear
 *     Bit 6 CHIE Channel Interrupt Enable
 *       Set for compare interrupt
 *     Bit 5 MSB Channel Mode Select B
 *       Set to 0
 *     Bit 4 MSA Channel Mode Select A
 *       Set to 1
 *     Bit 3:2  ELS Edge or Level Select
 *       Set to 0:0
 *   FTM1_COMBINE 
 *     Set WPDIS to 1 before writing
 *     DECAPEN (Dual Edge Capture Enable)
 *     COMBINE (Combine Channels)
 *     Resets to all zero
 *   FTM1_C0V Channel 0 Value
 *     Channel Compare Value
 *     Set to 0x80 - halfway thru count
 *   FTM1_STATUS 
 *     Duplicate of CHnF bit for all channels
 *     Bit 0 CH0F
 */

// Version 0.9 October 2014

#ifndef __QUADDECODE_H__
#define __QUADDECODE_H__

#include <stdint.h>
#include "mk20dx128.h"
#include "wiring.h"
#include "core_pins.h"


// Keep track of previous Interrupt to determine direction
enum prevInt_t{prevIntTOF, prevIntCompLow, prevIntCompHigh};

template <int N> class HardwareEncoder;

HardwareEncoder<1> *apQDcd1;	    // Pointers to  correct instance for ISR
HardwareEncoder<2> *apQDcd2;


template <int N>    // Valid values are 1 or 2 to set up as FTM1 or FTM2
class HardwareEncoder{
    private:
	float steps2Dist;
	float latestSteps2Dist;
	// Compare positions for 16 bit (64K) counter
	static const int NUM_64K=0x10000;	// Value of 16 bit counter
	static const uint16_t COMP1_4=NUM_64K/4;
	static const uint16_t COMP3_4=NUM_64K*3/4;
	static const uint16_t COMP3_8=NUM_64K*3/8;
	static const uint16_t COMP5_8=NUM_64K*5/8;

	static const uint16_t COMP_LOW=COMP3_8;	    // Lower compare point
	static const uint16_t COMP_HIGH=COMP5_8;    // Upper compare point
	// These values are also choosen past the compare points so we
	//   know if we are between the low and high value without a 
	//   compare interrupt we reversed direction
	static const uint16_t LOW_VALUE=COMP1_4;    // Closer to zero
	static const uint16_t HIGH_VALUE=COMP3_4;   // Closer to 64K

	// Bit select values
	static const uint32_t TOF=(1<<7);	// Bit 7 - Timer Overflow
	static const uint32_t TOFDIR=(1<<1);    // Bit 1
	static const uint32_t QUADDIR=(1<2);	// Bit 2
	static const uint32_t TOIE=(1<<6);	// Bit 6 - Int Enable

	// FTM module addresses set up as FTM1 or FTM 2 
	//   Then set up reference for ease of use
	volatile uint32_t * const pSC = 
	    reinterpret_cast<volatile uint32_t*>((N<2) ? 
		    0x40039000:0x400B8000);
	volatile uint32_t &FTM_SC=*pSC;	// Status And Control

	volatile uint32_t * const pCNT = 
	    reinterpret_cast<volatile uint32_t*>((N<2) ? 
		    0x40039004:0x400B8004);
	volatile uint32_t &FTM_CNT=*pCNT;	// Counter

	volatile uint32_t * const pMOD = 
	    reinterpret_cast<volatile uint32_t*>((N<2) ? 
		    0x40039008:0x400B8008);
	volatile uint32_t &FTM_MOD=*pMOD;	// Modulo

	volatile uint32_t * const pC0SC = 
	    reinterpret_cast<volatile uint32_t*>((N<2) ? 
		    0x4003900C:0x400B800C);
	volatile uint32_t &FTM_C0SC=*pC0SC;	// Channel 0 Status/Control

	volatile uint32_t * const pC0V = 
	    reinterpret_cast<volatile uint32_t*>((N<2) ? 
		    0x40039010:0x400B8010);
	volatile uint32_t &FTM_C0V=*pC0V;	// Channel 0 Value

	volatile uint32_t * const pC1SC = 
	    reinterpret_cast<volatile uint32_t*>((N<2) ? 
		    0x40039014:0x400B8014);
	volatile uint32_t &FTM_C1SC=*pC1SC;	// Channel 1 Status/Control

	volatile uint32_t * const pCNTIN = 
	    reinterpret_cast<volatile uint32_t*>((N<2) ? 
		    0x4003904C:0x400B804C);
	volatile uint32_t &FTM_CNTIN=*pCNTIN;   // Counter Initial Value

	volatile uint32_t * const pMODE = 
	    reinterpret_cast<volatile uint32_t*>((N<2) ? 
		    0x40039054:0x400B8054);
	volatile uint32_t &FTM_MODE=*pMODE;	// Features Mode Select

	volatile uint32_t * const pCOMBINE = 
	    reinterpret_cast<volatile uint32_t*>((N<2) ? 
		    0x40039064:0x400B8064);
	volatile uint32_t &FTM_COMBINE=*pCOMBINE;	// Linked Channels

	volatile uint32_t * const pFMS = 
	    reinterpret_cast<volatile uint32_t*>((N<2) ? 
		    0x40039074:0x400B8074);
	volatile uint32_t &FTM_FMS=*pFMS;	// Fault Mode Status

	volatile uint32_t * const pFILTER = 
	    reinterpret_cast<volatile uint32_t*>((N<2) ? 
		    0x40039078:0x400B8078);
	volatile uint32_t &FTM_FILTER=*pFILTER; // Input Filter Control

	volatile uint32_t * const pQDCTRL = 
	    reinterpret_cast<volatile uint32_t*>((N<2) ? 
		    0x40039080:0x400B8080);
	volatile uint32_t &FTM_QDCTRL=*pQDCTRL; // Quadrature Decode Control 

	// Variables
	volatile uint32_t v_read;   // Some variables have to be read before
				    //   they can be changed
	volatile int32_t v_basePosn; //Total position from counter overflows
	volatile bool v_initZeroTOF;	// First time thru after zero, 
					//  TOFDIR not valid
	volatile bool v_prevTOFUp;	// Direction on last overflow
	volatile prevInt_t v_prevInt;	
	volatile uint8_t v_intStatus;	// For interrupt routine debugging

    public:
	
	HardwareEncoder(){
	    // Order of contstructor execution not guaranteed, 
	    //   start with start()
	    if (N<2){	// Point to this instance for ISR
			apQDcd1=reinterpret_cast<HardwareEncoder<1>*>(this);
		// Without reintepret_cast compiler complains about unused 
		//   case.  Pointer for unused case not used, so don't
		//   care what it is.  Used case casts to existing type.
	    }else{
			apQDcd2=reinterpret_cast<HardwareEncoder<2>*>(this);
	    };
	};

	
	void setup(float stepsToDist){
		this->steps2Dist = steps2Dist;
		// Set up input pins
		if (N<2){	//FTM1
		// K20 pin 28,29
		// Bit 8-10 is Alt Assignment
		PORTA_PCR12 = 0x00000712;   //Alt7-QD_FTM1,FilterEnable,Pulldown
		PORTA_PCR13 = 0x00000712;   //Alt7-QD_FTM1,FilterEnable,Pulldown
		//Default is input, don't need to write PDDR (Direction)

		//Can also set up on K20 pin 35, 36 - not tested
		//PORTB_PCR1 = 0x00000612;   //Alt6-QD_FTM1,FilterEnable,Pulldown
		//PORTB_PCR1 = 0x00000612;   //Alt6-QD_FTM1,FilterEnable,Pulldown
		//Default is input, don't need to write PDDR (Direction)

		}else{  // FTM2
		// K20 pin 41,42
		// Bit 8-10 is Alt Assignment
		PORTB_PCR18 = 0x00000612;   //Alt6-QD_FTM2,FilterEnable,Pulldown
		PORTB_PCR19 = 0x00000612;   //Alt6-QD_FTM2,FilterEnable,Pulldown
		//Default is input, don't need to write PDDR (Direction)
		};

		//Set FTMEN to be able to write registers
		FTM_MODE=0x04;	    // Write protect disable - reset value
		FTM_MODE=0x05;	    // Set FTM Enable

		// Set registers written in pins_teensy.c back to default
		FTM_CNT = 0;
		FTM_MOD = 0;
		FTM_C0SC =0;
		FTM_C1SC =0;
		FTM_SC = 0;

		// Set registers to count quadrature
		FTM_FILTER=0x22;	// 2x4 clock filters on both channels
		FTM_CNTIN=0;
		FTM_MOD=0xFFFF;	// Maximum value of counter
		FTM_CNT=0;		// Updates counter with CNTIN

		// Set Registers for output compare mode
		FTM_COMBINE=0;	    // Reset value, make sure
		FTM_C0SC=0x10;	    // Bit 4 Channel Mode
		FTM_C0V= COMP_LOW;	    // Initial Compare Interrupt Value

		FTM_QDCTRL=0b11000001;	    // Quadrature control
		//        Filter enabled, QUADEN set
			
		// Write Protect Enable
		FTM_FMS=0x40;		// Write Protect, WPDIS=1
	}

	void start(){
		zeroFTM();	// Zero FTM counter

		// Enable global FTMx interrupt
		// Assume global interrupts already enabled 
		//   No individual FTM sources enabled yet
		
		if (N<2){	//FTM1
		NVIC_ENABLE_IRQ(IRQ_FTM1);
		} else {
		NVIC_ENABLE_IRQ(IRQ_FTM2);
		};
			
	}
	
	void zeroFTM(){

		// Turn off counter to disable interrupts and clear any 
		//   overflow and compare bits

		v_read=FTM_FMS;	// Requires reading before WPDIS can be set

		noInterrupts();
		FTM_MODE=0x05;	// Write Protect Disable with FTMEN set
		FTM_QDCTRL=0xC0;	// Disable QUADEN, filter still enabled
							//   Turns off counter so no interrupts
		interrupts();

		FTM_CNT=0;
		v_read=FTM_SC;	// Need to read Overflow before it can be set
		FTM_SC=0;		// Disable TOF interrupt, clear Overflow bit
		v_read=FTM_C0SC;	// Need to read Overflow before it can be set
		FTM_C0SC=0x10;	// Disable Channel compare int, clear compare bit
				//   Leaves Mode bit for compare set

	
		v_basePosn=0;
		v_initZeroTOF=true;	// Special case to determine direction 

		//v_prevInt=prevIntTOF;   // Value not used, remove code
		if (FTM_QDCTRL & QUADDIR){	//QuadDir says counting up
		FTM_C0V=COMP_LOW;	// Compare Interrupt Value
		} else {			//   according to last counting value
		FTM_C0V=COMP_HIGH;
		}

		// Enable counter again
		FTM_QDCTRL=0xC1;	// QUADEN
		FTM_FMS=0x40;  	// Write Protect, WPDIS=1
	
		//  Set channel interrupt
		FTM_C0SC=0x50;     // Enable Channel interrupt and Mode 

	}

	int32_t calcPosn(void){
		uint16_t count16;	// 16 bit position counter
		uint32_t count32;
		bool cpTOFDirUp;	// TOFDIR in QDCTRL (bit 1)
		int32_t cpBasePosn;
		int32_t currentPosn;
		bool dataConsistent;

		noInterrupts();	    // Variables can change if FTM interrrupt occurs
		if (!(FTM_SC & TOIE)){  // Check TOIE - TOF Interrupt disabled
		count16=FTM_CNT;    // Read current counter 
		count32=count16;
		// TOF Interrupt disabled so no change to basePosn
		if (count16 < LOW_VALUE){
			currentPosn=v_basePosn + count32;
			interrupts();   // Turn interrupts back on before return
			return currentPosn;
		}; 
		if ((count16 > HIGH_VALUE)){
			currentPosn=v_basePosn + count32 - NUM_64K;
			interrupts();   // Turn interrupts back on before return
			return currentPosn;
		}; 
			// Away from TOF, add or subtract depending on DIR last crossed TOF 
		if (v_initZeroTOF){	// No TOF since zero
			if (FTM_SC & TOF){  //TOF Set
			cpTOFDirUp=FTM_QDCTRL & TOFDIR;	// Away from TOF so stable
			} else {
			cpTOFDirUp = true;	// Reversal will cause TOF
						//   so counting up
			}
		}else{
			cpTOFDirUp=FTM_QDCTRL & TOFDIR; // Away from TOF so stable
		}
		// Use last TOFDIR to determine direction 
		if (cpTOFDirUp){
			currentPosn=v_basePosn + count32;
			interrupts();   // Turn interrupts back on before return
			return currentPosn;
		}; 
		if (!cpTOFDirUp){
			currentPosn=v_basePosn + count32 - NUM_64K;
			interrupts();   // Turn interrupts back on before return
			return currentPosn;
		};
		};
		// TOF Interrupt enabled
		interrupts();   // Allow TOF interrupt if pending
		count16=FTM_CNT;    // Read current counter 
		count32=count16;
		cpBasePosn=v_basePosn;  // Unchanging copy
		dataConsistent=(FTM_SC & TOIE);  // Still TOIE, no TOF
		// prevTOFUp set in Compare int that turns on TOIE
		//   Compare Int could occur but prevTOFUp should remain the same
		if (dataConsistent){
		// v_prevTOFUp changes in compare int, should be stable
		// Need to go thru compare int to set TOIE after zero
		if (v_prevTOFUp){	// Counting up
			currentPosn=cpBasePosn + count32;
		} else {
			currentPosn=cpBasePosn + count32 - NUM_64K;
		}
		return currentPosn;
		} else {	// Just crossed over TOF
		return v_basePosn;  // Position is new basePosition, no addtl counts
		}
	}
	float calcPosnF(void){
		return this->steps2Dist * (float)(calcPosn());
	}
	float calcSpeedF(long timeDelta){
		float distance = calcPosnF();
		float result = abs((distance-latestSteps2Dist)/timeDelta);
		latestSteps2Dist = distance;
		return result;
	}
	void ftm_isr(){
		
		//First determine which interrupt enabled
		if(FTM_SC == (TOF|TOIE)){	// TOIE and TOF set, TOF interrupt
		v_intStatus=1;

		v_initZeroTOF=false;	// TOF occured, TOFDIR valid

		// If previous direction and current same add or subtract 64K
		//   from base position
		// Otherwise reversed in mid-count, base position stays same
		FTM_SC = 0;	    // Clear TOIE, TOF don't care 
		FTM_C0SC= 0x50;	    // Set CHIE Channel Interrupt Enable
		if (v_prevInt==prevIntCompHigh){	// Counting up
			v_intStatus=110;
			FTM_C0V=COMP_LOW;	    // Set compare to closest value
			// SYNCEN in FTM1_COMBINE 0, update on next counter edge 
			if (v_prevTOFUp){
			v_intStatus=111;
			v_basePosn+=NUM_64K;    // Up for one counter cycle
			}
		} else {    // prevIntCompLow, counting down
			v_intStatus=100;
			FTM_C0V=COMP_HIGH;	// First expected compare counting down
			if (!v_prevTOFUp){
			v_intStatus=101;
			v_basePosn-=NUM_64K;
			}
		}   // Previous and current direction not the same
			//   indicate reversal, basePosn remains same
		}else { // Channel Compare Int
		v_intStatus=2;
		if (v_initZeroTOF){	// No TOF since zero
			if (FTM_SC & TOF){  //TOF Set
			v_initZeroTOF=false;	// TOF occured, TOFDIR valid
			} else {
			v_prevTOFUp = true;	// Reversal will cause TOF
						//   so counting up
			}
		}
		if (!v_initZeroTOF){	// TOFDIR valid
			v_prevTOFUp=FTM_QDCTRL & TOFDIR;	// Dir last crossed TOF
		}

		FTM_SC = TOIE;	// Should have been read in if statement
				// Can clear TOF, set TOIE 
		v_read=FTM_C0SC;   // Read to clear CHF Channel Flag
		FTM_C0SC= 0x50;    // Clear Channel Flag, leave CHIE set
		if (FTM_C0V==COMP_LOW){
			v_intStatus=30;
			v_prevInt=prevIntCompLow;
			FTM_C0V=COMP_HIGH;	// Set to other compare value
		} else {    // Compare at high value
			v_intStatus=40;
			v_prevInt=prevIntCompHigh;
			FTM_C0V=COMP_LOW;
		}	

		}
	}

};


void ftm1_isr(void){
    apQDcd1->ftm_isr();
};

void ftm2_isr(void){
    apQDcd2->ftm_isr();
};

#endif
