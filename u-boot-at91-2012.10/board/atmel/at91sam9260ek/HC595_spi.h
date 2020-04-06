


#ifndef __HC595_SPI_H__
#define __HC595_SPI_H__



#define HC595_BIT_LED11_RED           (1<<31)
#define HC595_BIT_LED11_GREEN         (1<<30)
#define HC595_BIT_LED12_RED           (1<<29)
#define HC595_BIT_LED12_GREEN         (1<<28)
#define HC595_BIT_LED13_RED           (1<<27)
#define HC595_BIT_LED13_GREEN         (1<<26)
#define HC595_BIT_LED14_RED           (1<<25)
#define HC595_BIT_LED14_GREEN         (1<<24)

#define HC595_BIT_LED10_RED           (1<<23)
#define HC595_BIT_LED10_GREEN         (1<<22)
#define HC595_BIT_LED9_RED            (1<<21)
#define HC595_BIT_LED9_GREEN          (1<<20)
#define HC595_BIT_LED16	              (1<<19)
#define HC595_BIT_LED15               (1<<18)
#define HC595_BIT_R249_Q21            (1<<17)
#define HC595_BIT_LCD_LIGHT           (1<<16)

#define HC595_BIT_LCD_PIN28           (1<<15)
#define HC595_BIT_CN8_PINn            (1<<14)
#define HC595_BIT_DM9161_RESET        (1<<13)
#define HC595_BIT_CN8_PINm            (1<<12)
#define HC595_BIT_K7                  (1<<11)
#define HC595_BIT_K6K5                (1<<10)
#define HC595_BIT_K4K3                (1<<9)
#define HC595_BIT_K2K1                (1<<8)

#define HC595_BIT_CN11_PINn           (1<<7)
#define HC595_BIT_IR1_PIN2            (1<<6)
#define HC595_BIT_CN11_PWR            (1<<5)
#define HC595_BIT_PC2_PULLUP          (1<<4)
#define HC595_BIT_LW051A_INH          (1<<3)
#define HC595_BIT_LW051A_C            (1<<2)
#define HC595_BIT_LW051A_B            (1<<1)
#define HC595_BIT_LW051A_A            (1<<0)



void HC595_SPI_Init(void);
void HC595_Set(unsigned int data, unsigned char updateFlag);
void HC595_set_bits(unsigned int bits, unsigned char updateFlag);
void HC595_clr_bits(unsigned int bits, unsigned char updateFlag);


#endif

