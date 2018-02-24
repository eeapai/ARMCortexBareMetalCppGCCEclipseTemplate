////////////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License                                                           //
//                                                                                //
// Copyright (c) 2018, pa.eeapai@gmail.com                                        //
// All rights reserved.                                                           //
//                                                                                //
// Redistribution and use in source and binary forms, with or without             //
// modification, are permitted provided that the following conditions are met:    //
//                                                                                //
// * Redistributions of source code must retain the above copyright notice, this  //
//   list of conditions and the following disclaimer.                             //
//                                                                                //
// * Redistributions in binary form must reproduce the above copyright notice,    //
//   this list of conditions and the following disclaimer in the documentation    //
//   and/or other materials provided with the distribution.                       //
//                                                                                //
// * Neither the name of the copyright holder nor the names of its                //
//   contributors may be used to endorse or promote products derived from         //
//   this software without specific prior written permission.                     //
//                                                                                //
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"    //
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE      //
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE //
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE   //
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL     //
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR     //
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER     //
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,  //
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE  //
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.           //
////////////////////////////////////////////////////////////////////////////////////
#include "stdio.h"

#define SWBKPT() { asm(" BKPT #0"); }

struct ISomeInterface
{
  virtual void Foonction() = 0;
};

class CSomeInterfaceImpl : public ISomeInterface // Polymorphism, beeyatch
{
  void Foonction() override
  {
    printf("Just like that\r\n");
  }
};

int main()
{
  printf("\33[2J\n\r");    // clear screen command
  printf("\33[3J\n\r");    // clear scroll back
  printf("Here I come, here I go\r\n"); // https://www.youtube.com/watch?v=9PSb2uYLCio

  CSomeInterfaceImpl si;
  ISomeInterface *psi = &si;
  while ( true )
  {
     printf("This will end up in PutChar\n\r");
     psi->Foonction();
  }
  return 0;
}

#ifdef __cplusplus
extern "C" {
#endif


/// Minimal sys impl
int PutChar(int ch)
{
  // Output to trace, UART, shared memory etc.
  return ch;
}

#include <sys/stat.h>

int _close(int file)
{
  return 0;
}

int _fstat(int file, struct stat *st)
{
  st->st_mode =  S_IFSOCK;//S_IFCHR;// S_IFIFO;
  return 0;
}

int _isatty(int file)
{
  return 1;
}

int _lseek(int file, int ptr, int dir)
{
  return 0;
}

int _read(int file, char *ptr, int len)
{
  return 0;
}

int _init()
{
  return 0;
}

caddr_t _sbrk(int incr)
{
  extern char _end;
  static char *heap_end;
  char *prev_heap_end;
  if (heap_end == 0)
  {
    heap_end = &_end;
  }
  prev_heap_end = heap_end;
  heap_end += incr;
  return (caddr_t) prev_heap_end;
}

int _write(int file, char *ptr, int len)
{
  int i;
  for ( i=0; i < len; i++ )
  {
    PutChar(*ptr++);
  }
  return len;
}
//////////////////////////////////////


typedef void (*pfunc)(void);

/* start address for the initialization values of the .data section defined in linker script */
extern unsigned long _sidata;
/* start address for the .data section. defined in linker script */
extern unsigned long _sdata;
/* end address for the .data section. defined in linker script */
extern unsigned long _edata;

/* start address for the .bss section. defined in linker script */
extern unsigned long _sbss;
/* end address for the .bss section. defined in linker script */
extern unsigned long _ebss;

extern pfunc _sinit_array[];
extern pfunc _einit_array[];

extern pfunc _sfinit_array[];
extern pfunc _efinit_array[];

int __cxa_atexit(void (*destroyer)(void*), void* object, void* dso_handle);
int __aeabi_atexit(void* object, void (*destroyer)(void*), void* dso_handle)
{
  return __cxa_atexit(destroyer, object, dso_handle);
}

void __cxa_pure_virtual(void)
{
  while (1);
}

void *__dso_handle;

int __cxa_atexit(void (*destructor) (void *), void *arg, void *dso)
{
  return 0;
}
void __cxa_finalize(void *f)
{

}

void operator delete(void *what)
{
  SWBKPT();   // No, no, no
  while( 1 );
}
__attribute__( ( naked ) )
void NMI_Handler()
{
  SWBKPT();
  while( 1 );
}

typedef struct
{
  unsigned long m_dwExR0;
  unsigned long m_dwExR1;
  unsigned long m_dwExR2;
  unsigned long m_dwExR3;
  unsigned long m_dwExR12;
  unsigned long m_dwExLR;
  unsigned long m_dwExPC;
  unsigned long m_dwExPSR;
}SARMv7MExceptionContext;

__attribute__( ( naked ) )
void HardFault_Handler()
{
  register SARMv7MExceptionContext * pARMv7MExceptionContext __asm("r0");
  register unsigned long dwLR __asm("lr");

  if ( dwLR & 4 )
    asm(" mrs r0, psp");
  else
    asm(" mrs r0, msp");

  SWBKPT();
  while ( 1 );
}
__attribute__( ( naked ) )
void MemManage_Handler()
{
  SWBKPT();
  while( 1 );
}

__attribute__( ( naked ) )
void BusFault_Handler()
{
  SWBKPT();
  while( 1 );
}

__attribute__( ( naked ) )
void UsageFault_Handler()
{
  SWBKPT();
  while( 1 );
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
}

__attribute__( ( naked ) )
void defaultHandler()
{
  SWBKPT();
  while( 1 );
}

__attribute__( ( naked ) )
void WWDG_IRQHandler()
{
  SWBKPT();
  while( 1 );
}

__attribute__( ( naked ) )
void PVD_IRQHandler()
{
  SWBKPT();
  while( 1 );
}


#if 1
/* Copy the data segment initializers from flash to SRAM */
static void copyData(void)
{
  volatile unsigned long *pdwDataSrc = &_sidata;
  volatile unsigned long *pdwDataDest = &_sdata;

  if ( &_sdata == &_edata )
  {
   return;
  }

  if ( &_sidata == &_sdata )
  {
   return;
  }

  while( pdwDataDest < &_edata )
  {
    *pdwDataDest++ = *pdwDataSrc++;
  }
}

/* Zero fill the bss segment. */
static void zeroBss(void)
{
  volatile unsigned long *pdwDataDest = &_sbss;

  if ( &_sbss == &_ebss )
  {
   return;
  }

  while ( pdwDataDest < &_ebss )
  {
    *pdwDataDest++ = 0;
  }
}

/* Call static constructors */
static void callConstructors(void)
{
  pfunc *pfTable;
  if ( _sinit_array == _einit_array )
  {
   return;
  }

  for ( pfTable = _einit_array - 1; pfTable - _sinit_array >= 0; pfTable-- )
  {
    pfTable[0]();
  }
}

/* Call static destructors */

//static void callDestructors(void)
//{
//  pfunc *pfTable;
//  if ( _sfinit_array == _efinit_array )
//  {
//   return;
//  }
//
//  for ( pfTable = _efinit_array - 1; pfTable - _sfinit_array >= 0; pfTable-- )
//  {
//    pfTable[0]();
//  }
//}
void _exit (int a)
{
  while(1) {};
}

extern const unsigned long g_adwVectors[];
extern unsigned long _estack;
__attribute__( ( naked ) )
void Reset_Handler(void)
{
  register unsigned long *pulSP __asm("sp") = &_estack;
  copyData();
  zeroBss();
  callConstructors();
//  atexit();
//  _init();
//  SystemInit();
  main();
  *pulSP = g_adwVectors[0];
//  exit();
  SWBKPT();
  while( 1 );
}

__attribute__ ((section(".isr_vector")))
const unsigned long g_adwVectors[] =
{
  (unsigned long)&_estack                     ,
  (unsigned long)Reset_Handler                ,
  (unsigned long)NMI_Handler                  ,
  (unsigned long)HardFault_Handler            ,
  (unsigned long)MemManage_Handler            ,
  (unsigned long)BusFault_Handler             ,
  (unsigned long)UsageFault_Handler           ,
  (unsigned long) 0                           ,
  (unsigned long) 0                           ,
  (unsigned long) 0                           ,
  (unsigned long) 0                           ,
  (unsigned long) SVC_Handler                 ,
  (unsigned long) DebugMon_Handler            ,
  (unsigned long) 0                           ,
  (unsigned long) PendSV_Handler              ,
  (unsigned long) SysTick_Handler             ,
 };
#endif

#ifdef __cplusplus
}
#endif
