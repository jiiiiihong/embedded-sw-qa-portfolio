#include <stm32f10x.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define QUEUE_SIZE 11
#define ADC1_DR_BASE	0x4001244C


typedef struct {
    int data[QUEUE_SIZE];
    int front;
    int rear;
    int count;
		int value;
} Queue;

void initQueue(Queue *q) {
    q->front = 0;
    q->rear = -1;
    q->count = 0;
    q->value = 0;
    for (int t = 0; t < QUEUE_SIZE; t++) {
        q->data[t] = 0;
    }
}

bool isQueueEmpty(Queue *q) {
    return (q->count == 0);
}

bool isQueueFull(Queue *q) {
    return (q->count == QUEUE_SIZE);
}

bool enqueue(Queue *q, int value) {
    if (isQueueFull(q)) {
        return false;
    } else {
        q->rear = (q->rear + 1) % QUEUE_SIZE;
        q->data[q->rear] = value;
        q->count++;
        return true;
    }
}

bool dequeue(Queue *q) {
    if (isQueueEmpty(q)) {
        return false;
    } else {
				q->value = q->data[q->front];
        q->front = (q->front + 1) % QUEUE_SIZE;
        q->count--;
        return true;
    }
}

Queue myQueue;
Queue time_set_que;
Queue passcode_que;
Queue my_passcode_que;
Queue r_data_que;

u32 font4x8[10][8] = {
 {0x00,0xe0,0xa0,0xa0,0xa0,0xa0,0xe0,0x00}, // 0
 {0x00,0x40,0x60,0x40,0x40,0x40,0xe0,0x00}, // 1
 {0x00,0xc0,0xa0,0x80,0x40,0x20,0xe0,0x00}, // 2
 {0x00,0xe0,0x80,0x80,0xe0,0x80,0xe0,0x00}, // 3
 {0x00,0xa0,0xa0,0xa0,0xe0,0x80,0x80,0x00}, // 4
 {0x00,0xe0,0x20,0x20,0xe0,0x80,0xe0,0x00}, // 5
 {0x00,0xc0,0x20,0x20,0xe0,0xa0,0xe0,0x00}, // 6
 {0x00,0xe0,0xa0,0xa0,0x80,0x80,0x80,0x00}, // 7
 {0x00,0xe0,0xa0,0xa0,0xe0,0xa0,0xe0,0x00}, // 8
 {0x00,0xe0,0xa0,0xa0,0xe0,0x80,0xe0,0x00}  // 9
};

u32 font_dotdot[8] = {0,0,1,0,0,1,0,0};
u8 font_dot[1][8] = {1,0,0,0,0,0,0,0};
u8 font_dot_temp[1][8] = {0,0,0,0,0,0,1,0};
u32 font_timeout[1][8] = {0x00, 0x7e, 0x5a, 0x18, 0x18, 0x18, 0x3c, 0x00};
u32 font_p[1][8] = {0x00, 0x7e, 0x66, 0x7e, 0x06, 0x06, 0x06, 0x00};
u32 font_O[1][8] = {0x00, 0x3c, 0x42, 0x42, 0x42, 0x42, 0x3c, 0x00};
u32 font_X[1][8] = {0x00, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x00};
u32 font_D[1][8] = {0x00, 0x3e, 0x42, 0x42, 0x42, 0x42, 0x3e, 0x00};

u8 data, fail_3;
u32 row, col, blink, i, j, k, p, q, r, m, n, a, b, c, tt, tt_b;
u32 i_f, k_f, m_f, n_f, a_f, b_f;
u8 key_index;
u32 key_row, key_col, col_scan;

u16 ADCConverted;
u32 t_10, t_1, t_1_1;
volatile float temperature;
u32 pass_set_time;
u32 passcode[8] = {0,0,0,0,0,0,0,0};
u32 kkk, kkkk;
u32 xx;
u32 kkk_t, kkkk_t;
u32 kkk_tt, kkkk_tt;
u32 enter;
u8 ox_mode;
u8 mismatch;
u8 ox_t;
u8 data_r;
int main (void) {
   
RCC->APB2ENR = 0x0000481F;
RCC->APB2ENR |= 0x000040004;
RCC->APB1ENR |= 0x10000007;
RCC->AHBENR |= 0x00000001;
AFIO->MAPR |= 0x2 << 24;
PWR->CR |= 1<<8;
RCC->BDCR &= ~(1<<0);
	
    while (RCC->BDCR & 0x2) {
        // Wait until LSE oscillator is disabled
    }	
GPIOA->CRL = 0x44404444;
GPIOA->CRH &= ~(0xFFu << 4);
GPIOA->CRH |= (0x04B << 4);
		
GPIOC->CRL = 0x33333333;  // PC[7:0] => dot matrix col[7:0]
GPIOC->CRH = 0x33333333;  // PC[8:15] => dot matrix col[8:15]	
GPIOB->CRL = 0x33333333;  // PB[0:7] => dot matrix row[0:7]

GPIOB->CRH = 0x33338888; // keypad row output: PB[15:12], keypad col input with pull-up: PB[11:8]
GPIOB->ODR = 0x0F00;

				DMA1_Channel1->CCR = 0x00003520;
				DMA1_Channel1->CNDTR = 1;
				DMA1_Channel1->CPAR = ADC1_DR_BASE;
				DMA1_Channel1->CMAR = (u32)&ADCConverted;

				RCC->APB2ENR |= 0x00000200;
				ADC1->CR1 = 0x00000000;
				ADC1->CR2 = 0x001E0102;
				ADC1->SMPR2 = 0x00007000;
				ADC1->SQR1 = 0x00000000;
				ADC1->SQR2 = 0x00000000;
				ADC1->SQR3 = 0x00000004;
	
				DMA1_Channel1->CCR |= 0x00000001;
				ADC1->CR2 |= 0x00000001;
				ADC1->CR2 |= 0x00400000;		
   
         TIM1->CR1 = 0x00;
         TIM1->CR2 = 0x00;
         TIM1->PSC = 99;
         TIM1->ARR = 999;
   
         TIM1->DIER = 0x0001;
         NVIC->ISER[0] = 0x02000000;
         TIM1->CR1 |= 0x0001;
   
         TIM2->CR1 = 0x00;
         TIM2->CR2 = 0x00;
         TIM2->PSC = 7199;
         TIM2->ARR = 4999;
   
         TIM2->DIER = 0x0001;
         NVIC->ISER[0] |= 0x10000000;
         TIM2->CR1 |= 0x0001;
             
         TIM3->CR1 = 0x00;
         TIM3->CR2 = 0x00;
         TIM3->PSC = 7199;
         TIM3->ARR = 9999;
         
         TIM3->DIER = 0x0001;
         NVIC->ISER[0] |= 0x20000000;
         TIM3->CR1 |= 0x0001;
             
         TIM4->CR1 = 0x00;
         TIM4->CR2 = 0x00;
         TIM4->PSC = 199;
         TIM4->ARR = 99;
         
         TIM4->DIER = 0x0001;
         NVIC->ISER[0] |= 0x40000000;
         TIM4->CR1 |= 0x0001; 

				 USART1->BRR = 0x0FFF;
				 USART1->CR1 = 0x00000000;
				 USART1->CR2 = 0x00000000;
				 USART1->CR3 = 0x00000000;
				 USART1->CR1 |= 0x00000004;
				 USART1->CR1 |= 0x00002000;
				 
				 NVIC->ISER[1] |= (1 << 5);
				 USART1->CR1 |= 0x00000020;
   
         
         row = 1;
         col = 0;
         i = 0;
         j = 0;
         k = 0;
         m = 0;
         n = 0;
         a = 2;
         b = 1;
				 c = 12;
         p = 0;
				 q = 0;
				 r = 0;
				 key_index = 0;
				 key_row = 0x01;
				 initQueue(&myQueue);	
				 initQueue(&time_set_que);
				 initQueue(&passcode_que);
				 initQueue(&my_passcode_que);
				 initQueue(&r_data_que);
				 i_f = 0;
				 k_f = 0;
				 m_f = 0;
				 n_f = 0;
				 a_f = 2;
				 b_f = 1;
				 tt = 0;
				 tt_b = 0;
				 kkk = 0;
				 kkkk = 0;
				 xx = 0;
				 kkk_t = 0;
				 kkkk_t = 0;
				 kkk_tt = 0;
				 kkkk_tt = 0;
				 enter = 0;
				 ox_mode = 0;
				 mismatch = 0;
        //b>a>n>m>k>i
				 fail_3 = 0;
				 pass_set_time = 0;
				 ox_t = 0;
         while(1){;}
}

void TIM1_UP_IRQHandler(void) {
    if (TIM1->SR & 0x0001) {
        GPIOB->BSRR = ~row | ((row & 0xFF) << 16);
			if((myQueue.count == 0)&&(time_set_que.count == 0)&&(kkk==0)&&(kkkk==0)&&(ox_mode==0)){  //default mode, kkk:passcode setup mode, kkkk:passcode inserting mode
				if(q%4==0|q%4==1)
					GPIOC->ODR = font_dot[0][j]<<13;
				else
					GPIOC->ODR = 0;
				if(passcode_que.data[passcode_que.rear] != 12) initQueue(&passcode_que);   //reset passcode if last data is not #
				//if((my_passcode_que.data[my_passcode_que.rear] != 12)|(my_passcode_que.count<=5)) initQueue(&my_passcode_que);   //reset my_passcode if last data is not # or insrted number is under 5
				initQueue(&my_passcode_que);
			}
			if((time_set_que.count == 0)&&(kkk==0)&&(kkkk==0)&&(ox_mode==0)){
				if(myQueue.count >= 1){   //time show
					col = font4x8[i][j]<<12 | (font4x8[k][j])<<9 | font_dotdot[j]<<13 | (font4x8[m][j])<<5 | (font4x8[n][j] << 2) | font_dotdot[j]<<6 | (font4x8[a][j])>>2 | (font4x8[b][j]>>5);
					col >>= 6;				
					if(p == 1)
						col<<=0;
					else if(p == 2){
						col<<=1;
						col |= (font4x8[a][j])>>7;
					}
					else if(p == 3){
						col<<=2;
						col |= (font4x8[a][j])>>6;
					}
					else if(p == 4){
						col<<=3;
						col |= (font4x8[a][j])>>5;
					}
					else if(p == 5){
						col<<=4;
						col |= (font4x8[a][j])>>4|(font4x8[b][j]>>7);
					}
					else if(p == 6){
						col<<=5;
						col |= (font4x8[a][j])>>3|(font4x8[b][j]>>6);
					}
					else if(p == 7){
						col<<=6;
						col |= (font4x8[a][j])>>2|(font4x8[b][j]>>5);
					}
					else if(p == 8){
						col = font4x8[t_1_1][j]<<5|font_dot_temp[0][j]<<8|font4x8[t_1][j]|font4x8[t_10][j]>>5;
					}
					else if(p == 9){
						dequeue(&myQueue);
					}					
								
					GPIOC->ODR = col;
				}				
			}
			else if((time_set_que.count > 0)&&(kkk==0)&&(kkkk==0)&&(ox_mode==0)){
					if(q%2 ==0){
							col = font4x8[i_f][j]<<12 | (font4x8[k_f][j])<<9 | font_dotdot[j]<<13 | (font4x8[m_f][j])<<5 | (font4x8[n_f][j] << 2) | font_dotdot[j]<<6 | (font4x8[a_f][j])>>2 | (font4x8[b_f][j]>>5);
							col >>= 6;				
							if(r == 1)
								col<<=0;
							else if(r == 2){
								col<<=1;
								col |= (font4x8[a_f][j])>>7;
							}
							else if(r == 3){
								col<<=2;
								col |= (font4x8[a_f][j])>>6;
							}
							else if(r == 4){
								col<<=3;
								col |= (font4x8[a_f][j])>>5;
							}
							else if(r == 5){
								col<<=4;
								col |= (font4x8[a_f][j])>>4|(font4x8[b_f][j]>>7);
							}
							else if(r == 6){
								col<<=5;
								col |= (font4x8[a_f][j])>>3|(font4x8[b_f][j]>>6);
							}
							else if(r == 7){
								col<<=6;
								col |= (font4x8[a_f][j])>>2|(font4x8[b_f][j]>>5);
							}
									
							GPIOC->ODR = col;			
					}	
					else GPIOC->ODR = 0;	
				  
					if(time_set_que.count == 2){
							if(tt>4) {
								GPIOC->ODR = font_timeout[0][j];
							}
							else{
								k = data_r;
								k_f = k;
								if(k >= 6) {
									k = 0;
									k_f = 0;
								}
							}
					}
					else if(time_set_que.count == 3){
							if(tt>4){
								GPIOC->ODR = font_timeout[0][j];
							}
							else{
								i = data_r;
								i_f = i;
								if(i >= 10){
									i = 0;
									i_f = 0;
								}								
							}							
					}	
					else if(time_set_que.count == 4){
							if(tt>4){
								GPIOC->ODR = font_timeout[0][j];
							}
							else{
								n = data_r;
								n_f = n;
								if(n >= 6){
									n = 0;
									n_f = 0;
							}
						}
												
					}						 
					else if(time_set_que.count == 5){
							if(tt>4){
								GPIOC->ODR = font_timeout[0][j];
							}
							else{
								m = data_r;
								m_f = m;
								if(m >= 10){
									m = 0;
									m_f = 0;
								}								
							}						
					}						
					else if(time_set_que.count == 6){
						if(tt>4){
							GPIOC->ODR = font_timeout[0][j];
						}
						else{
							b = data_r;
							b_f = b;
							if(b >= 3){
								b = 0;
								b_f = 0;
							}							
						}
					}						
					else if(time_set_que.count == 7) {
						if(tt>4){
							GPIOC->ODR = font_timeout[0][j];
						}
						else{
							a = data_r;
							a_f = a;
							if( b == 2 && a >= 4){
								a = 0;
								b = 0;
								a_f = 0;
								b_f = 0;
							}
							else if( a >= 10) {
								a = 0;
								a_f = 0;
							}
							dequeue(&time_set_que);
							dequeue(&time_set_que);
							dequeue(&time_set_que);
							dequeue(&time_set_que);
							dequeue(&time_set_que);
							dequeue(&time_set_que);
							dequeue(&time_set_que);							
						}					
				}
			}
			
			else if((time_set_que.count == 0)&&(kkk==1)&&(kkkk==0)&&(ox_mode==0)){       //passcode setup mode display
				if(kkk_t <= 2){
					GPIOC->ODR = font_p[0][j];					
				}
				else if(kkk_t == 3){    //start show passcode:0s
					col = font4x8[passcode_que.data[2]][j]>>3|font4x8[passcode_que.data[3]][j]<<1|font4x8[passcode_que.data[4]][j]<<5;  //right font moves leftward 4 bits more than left font
					GPIOC->ODR = col;
					if(kkk_tt>=2){
						GPIOC->ODR = font_timeout[0][j];
					}	
				}
				else if(kkk_t == 4){  //1s-1
					if(passcode_que.data[1] >= 4){
						col = font4x8[passcode_que.data[2]][j]>>4|font4x8[passcode_que.data[3]][j]|font4x8[passcode_que.data[4]][j]<<4|font4x8[passcode_que.data[5]][j]<<8;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}						
					}
					else{
						kkk=0;
						kkk_t=0;
					}
				}
				else if(kkk_t == 5){  //2s
					if(passcode_que.data[1] >= 4){
						col = font4x8[passcode_que.data[2]][j]>>5|font4x8[passcode_que.data[3]][j]>>1|font4x8[passcode_que.data[4]][j]<<3|font4x8[passcode_que.data[5]][j]<<7;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}						
					}
					else{
						kkk=0;
						kkk_t=0;
					}					
				}
				else if(kkk_t == 6){  //3s
					if(passcode_que.data[1] >= 4){
						col = font4x8[passcode_que.data[2]][j]>>6|font4x8[passcode_que.data[3]][j]>>2|font4x8[passcode_que.data[4]][j]<<2|font4x8[passcode_que.data[5]][j]<<6;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
						
					}
					else{
						kkk=0;
						kkk_t=0;
					}						
				}
				else if(kkk_t == 7){   //last snapshot of MSV : 4s
					if(passcode_que.data[1] >= 4){
						col = font4x8[passcode_que.data[2]][j]>>7|font4x8[passcode_que.data[3]][j]>>3|font4x8[passcode_que.data[4]][j]<<1|font4x8[passcode_que.data[5]][j]<<5;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else{
						kkk=0;
						kkk_t=0;
					}					
				}
				else if(kkk_t == 8){  //1s-2
					if(passcode_que.data[1] >= 5){
						col = font4x8[passcode_que.data[3]][j]>>4|font4x8[passcode_que.data[4]][j]|font4x8[passcode_que.data[5]][j]<<4|font4x8[passcode_que.data[6]][j]<<8;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else{
						kkk=0;
						kkk_t=0;
					}					
				}
				else if(kkk_t == 9){  //2s
					if(passcode_que.data[1] >= 5){
						col = font4x8[passcode_que.data[3]][j]>>5|font4x8[passcode_que.data[4]][j]>>1|font4x8[passcode_que.data[5]][j]<<3|font4x8[passcode_que.data[6]][j]<<7;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else {
						kkk=0;
						kkk_t=0;
					}
				}
				else if(kkk_t == 10){  //3s
					if(passcode_que.data[1] >= 5){
						col = font4x8[passcode_que.data[3]][j]>>6|font4x8[passcode_que.data[4]][j]>>2|font4x8[passcode_que.data[5]][j]<<2|font4x8[passcode_que.data[6]][j]<<6;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else {
						kkk=0;
						kkk_t=0;
					}
				}
				else if(kkk_t == 11){  //4s
					if(passcode_que.data[1] >= 5){
						col = font4x8[passcode_que.data[3]][j]>>7|font4x8[passcode_que.data[4]][j]>>3|font4x8[passcode_que.data[5]][j]<<1|font4x8[passcode_que.data[6]][j]<<5;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else {
						kkk=0;
						kkk_t=0;
					}
				}
				else if(kkk_t == 12){  //1s-3
					if(passcode_que.data[1] >= 6){
						col = font4x8[passcode_que.data[4]][j]>>4|font4x8[passcode_que.data[5]][j]|font4x8[passcode_que.data[6]][j]<<4|font4x8[passcode_que.data[7]][j]<<8;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else {
						kkk=0;
						kkk_t=0;
					}
				}
				else if(kkk_t == 13){  //2s
					if(passcode_que.data[1] >= 6){
						col = font4x8[passcode_que.data[4]][j]>>5|font4x8[passcode_que.data[5]][j]>>1|font4x8[passcode_que.data[6]][j]<<3|font4x8[passcode_que.data[7]][j]<<7;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else {
						kkk=0;
						kkk_t=0;
					}
				}
				else if(kkk_t == 14){  //3s
					if(passcode_que.data[1] >= 6){
						col = font4x8[passcode_que.data[4]][j]>>6|font4x8[passcode_que.data[5]][j]>>2|font4x8[passcode_que.data[6]][j]<<2|font4x8[passcode_que.data[7]][j]<<6;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else {
						kkk=0;
						kkk_t=0;
					}
				}
				else if(kkk_t == 15){  //4s
					if(passcode_que.data[1] >= 6){
						col = font4x8[passcode_que.data[4]][j]>>7|font4x8[passcode_que.data[5]][j]>>3|font4x8[passcode_que.data[6]][j]<<1|font4x8[passcode_que.data[7]][j]<<5;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else {
						kkk=0;
						kkk_t=0;
					}
				}
				else if(kkk_t == 16){ //1s-4
					if(passcode_que.data[1] >= 7){
						col = font4x8[passcode_que.data[5]][j]>>4|font4x8[passcode_que.data[6]][j]|font4x8[passcode_que.data[7]][j]<<4|font4x8[passcode_que.data[8]][j]<<8;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else {
						kkk=0;
						kkk_t=0;
					}
				}
				else if(kkk_t == 17){  //2s
					if(passcode_que.data[1] >= 7){
						col = font4x8[passcode_que.data[5]][j]>>5|font4x8[passcode_que.data[6]][j]>>1|font4x8[passcode_que.data[7]][j]<<3|font4x8[passcode_que.data[8]][j]<<7;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else {
						kkk=0;
						kkk_t=0;
					}
				}
				else if(kkk_t == 18){  //3s
					if(passcode_que.data[1] >= 7){
						col = font4x8[passcode_que.data[5]][j]>>6|font4x8[passcode_que.data[6]][j]>>2|font4x8[passcode_que.data[7]][j]<<2|font4x8[passcode_que.data[8]][j]<<6;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else {
						kkk=0;
						kkk_t=0;
					}
				}
				else if(kkk_t == 19){  //4s
					if(passcode_que.data[1] >= 7){
						col = font4x8[passcode_que.data[5]][j]>>7|font4x8[passcode_que.data[6]][j]>>3|font4x8[passcode_que.data[7]][j]<<1|font4x8[passcode_que.data[8]][j]<<5;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else {
						kkk=0;
						kkk_t=0;
					}
				}
				else if(kkk_t == 20){  //1s-5
					if(passcode_que.data[1] >= 8){
						col = font4x8[passcode_que.data[6]][j]>>4|font4x8[passcode_que.data[7]][j]|font4x8[passcode_que.data[8]][j]<<4|font4x8[passcode_que.data[9]][j]<<8;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else {
						kkk=0;
						kkk_t=0;
					}
				}
				else if(kkk_t == 21){  //2s
					if(passcode_que.data[1] >= 8){
						col = font4x8[passcode_que.data[6]][j]>>5|font4x8[passcode_que.data[7]][j]>>1|font4x8[passcode_que.data[8]][j]<<3|font4x8[passcode_que.data[9]][j]<<7;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else {
						kkk=0;
						kkk_t=0;
					}
				}
				else if(kkk_t == 22){  //3s
					if(passcode_que.data[1] >= 8){
						col = font4x8[passcode_que.data[6]][j]>>6|font4x8[passcode_que.data[7]][j]>>2|font4x8[passcode_que.data[8]][j]<<2|font4x8[passcode_que.data[9]][j]<<6;
						GPIOC->ODR = col;
						if(kkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else {
						kkk=0;
						kkk_t=0;
					}
				}							
				else {
					kkk=0;
					kkk_t=0;
				}									
			}
			else if((time_set_que.count == 0)&&(kkk==0)&&(kkkk==1)&&(ox_mode==0)){     //passcode guess mode display
				if(kkkk_t <= 1){					//start show passcode:0s
						col = font4x8[my_passcode_que.data[1]][j]>>3|font4x8[my_passcode_que.data[2]][j]<<1|font4x8[my_passcode_que.data[3]][j]<<5;  //right font moves leftward 4 bits more than left font
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}								
				}
				else if(kkkk_t == 2){  //1s-1
						col = font4x8[my_passcode_que.data[1]][j]>>4|font4x8[my_passcode_que.data[2]][j]|font4x8[my_passcode_que.data[3]][j]<<4|font4x8[my_passcode_que.data[4]][j]<<8;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}				
				}
				else if(kkkk_t == 3){   //2s
						col = font4x8[my_passcode_que.data[1]][j]>>5|font4x8[my_passcode_que.data[2]][j]>>1|font4x8[my_passcode_que.data[3]][j]<<3|font4x8[my_passcode_que.data[4]][j]<<7;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}					
				}
				else if(kkkk_t == 4){   //3s
						col = font4x8[my_passcode_que.data[1]][j]>>6|font4x8[my_passcode_que.data[2]][j]>>2|font4x8[my_passcode_que.data[3]][j]<<2|font4x8[my_passcode_que.data[4]][j]<<6;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}					
				}
				else if(kkkk_t == 5){   //4s
						col = font4x8[my_passcode_que.data[1]][j]>>7|font4x8[my_passcode_que.data[2]][j]>>3|font4x8[my_passcode_que.data[3]][j]<<1|font4x8[my_passcode_que.data[4]][j]<<5;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
					  }				
				}
				else if(kkkk_t == 6){  //1s-2
					if((enter == 1) && (my_passcode_que.rear >= 6)){
						col = font4x8[my_passcode_que.data[2]][j]>>4|font4x8[my_passcode_que.data[3]][j]|font4x8[my_passcode_que.data[4]][j]<<4|font4x8[my_passcode_que.data[5]][j]<<8;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else{
						kkkk=0;
						kkkk_t=0;
						ox_mode = 1;
					}				
				}
				else if(kkkk_t == 7){   //2s
					if((enter == 1) && (my_passcode_que.rear >= 6)){
						col = font4x8[my_passcode_que.data[2]][j]>>5|font4x8[my_passcode_que.data[3]][j]>>1|font4x8[my_passcode_que.data[4]][j]<<3|font4x8[my_passcode_que.data[5]][j]<<7;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else{
						kkkk=0;
						kkkk_t=0;
						ox_mode = 1;
					}
					
				}
				else if(kkkk_t == 8){   //3s
					if((enter == 1) && (my_passcode_que.rear >= 6)){
						col = font4x8[my_passcode_que.data[2]][j]>>6|font4x8[my_passcode_que.data[3]][j]>>2|font4x8[my_passcode_que.data[4]][j]<<2|font4x8[my_passcode_que.data[5]][j]<<6;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else{
						kkkk=0;
						kkkk_t=0;
						ox_mode = 1;
					}				
				}
				else if(kkkk_t == 9){   //4s
					if((enter == 1) && (my_passcode_que.rear >= 6)){
						col = font4x8[my_passcode_que.data[2]][j]>>7|font4x8[my_passcode_que.data[3]][j]>>3|font4x8[my_passcode_que.data[4]][j]<<1|font4x8[my_passcode_que.data[5]][j]<<5;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
					  }
					}
					else{
						kkkk=0;
						kkkk_t=0;
						ox_mode = 1;
					}				
				}
				else if(kkkk_t == 10){  //1s-3
					if((enter == 1) && (my_passcode_que.rear >= 7)){
						col = font4x8[my_passcode_que.data[3]][j]>>4|font4x8[my_passcode_que.data[4]][j]|font4x8[my_passcode_que.data[5]][j]<<4|font4x8[my_passcode_que.data[6]][j]<<8;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else{
						kkkk=0;
						kkkk_t=0;
						ox_mode = 1;
						xx=1;
					}			
					
				}
				else if(kkkk_t == 11){   //2s
					if((enter == 1) && (my_passcode_que.rear >= 7)){
						col = font4x8[my_passcode_que.data[3]][j]>>5|font4x8[my_passcode_que.data[4]][j]>>1|font4x8[my_passcode_que.data[5]][j]<<3|font4x8[my_passcode_que.data[6]][j]<<7;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else{
						kkkk=0;
						kkkk_t=0;
						ox_mode = 1;
					}			
					
				}
				else if(kkkk_t == 12){   //3s
					if((enter == 1) && (my_passcode_que.rear >= 7)){
						col = font4x8[my_passcode_que.data[3]][j]>>6|font4x8[my_passcode_que.data[4]][j]>>2|font4x8[my_passcode_que.data[5]][j]<<2|font4x8[my_passcode_que.data[6]][j]<<6;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else{
						kkkk=0;
						kkkk_t=0;
						ox_mode = 1;
					}					
				}
				else if(kkkk_t == 13){   //4s
					if((enter == 1) && (my_passcode_que.rear >= 7)){
						col = font4x8[my_passcode_que.data[3]][j]>>7|font4x8[my_passcode_que.data[4]][j]>>3|font4x8[my_passcode_que.data[5]][j]<<1|font4x8[my_passcode_que.data[6]][j]<<5;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
					  }
				  }
					else{
						kkkk=0;
						kkkk_t=0;
						ox_mode = 1;
					}					
				}
				else if(kkkk_t == 14){  //1s-4
					if((enter == 1) && (my_passcode_que.rear >= 8)){
						col = font4x8[my_passcode_que.data[4]][j]>>4|font4x8[my_passcode_que.data[5]][j]|font4x8[my_passcode_que.data[6]][j]<<4|font4x8[my_passcode_que.data[7]][j]<<8;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else{
						kkkk=0;
						kkkk_t=0;
						ox_mode = 1;
					}					
				}
				else if(kkkk_t == 15){   //2s
					if((enter == 1) && (my_passcode_que.rear >= 8)){
						col = font4x8[my_passcode_que.data[4]][j]>>5|font4x8[my_passcode_que.data[5]][j]>>1|font4x8[my_passcode_que.data[6]][j]<<3|font4x8[my_passcode_que.data[7]][j]<<7;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else{
						kkkk=0;
						kkkk_t=0;
						ox_mode = 1;
					}					
				}
				else if(kkkk_t == 16){   //3s
					if((enter == 1) && (my_passcode_que.rear >= 8)){
						col = font4x8[my_passcode_que.data[4]][j]>>6|font4x8[my_passcode_que.data[5]][j]>>2|font4x8[my_passcode_que.data[6]][j]<<2|font4x8[my_passcode_que.data[7]][j]<<6;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else{
						kkkk=0;
						kkkk_t=0;
						ox_mode = 1;
					}					
				}
				else if(kkkk_t == 17){   //4s
					if((enter == 1) && (my_passcode_que.rear >= 8)){
						col = font4x8[my_passcode_que.data[4]][j]>>7|font4x8[my_passcode_que.data[5]][j]>>3|font4x8[my_passcode_que.data[6]][j]<<1|font4x8[my_passcode_que.data[7]][j]<<5;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
					  }
				  }
					else{
						kkkk=0;
						kkkk_t=0;
						ox_mode = 1;
					}					
				}
				else if(kkkk_t == 18){  //1s-5
					if((enter == 1) && (my_passcode_que.rear >= 9)){
						col = font4x8[my_passcode_que.data[5]][j]>>4|font4x8[my_passcode_que.data[6]][j]|font4x8[my_passcode_que.data[7]][j]<<4|font4x8[my_passcode_que.data[8]][j]<<8;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else{
						kkkk=0;
						kkkk_t=0;
						ox_mode = 1;
					}					
				}
				else if(kkkk_t == 19){   //2s
					if((enter == 1) && (my_passcode_que.rear >= 9)){
						col = font4x8[my_passcode_que.data[5]][j]>>5|font4x8[my_passcode_que.data[6]][j]>>1|font4x8[my_passcode_que.data[7]][j]<<3|font4x8[my_passcode_que.data[8]][j]<<7;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else{
						kkkk=0;
						kkkk_t=0;
						ox_mode = 1;
					}					
				}
				else if(kkkk_t == 20){   //3s
					if((enter == 1) && (my_passcode_que.rear >= 9)){
						col = font4x8[my_passcode_que.data[5]][j]>>6|font4x8[my_passcode_que.data[6]][j]>>2|font4x8[my_passcode_que.data[7]][j]<<2|font4x8[my_passcode_que.data[8]][j]<<6;
						GPIOC->ODR = col;
						if(kkkk_tt>=2){
							GPIOC->ODR = font_timeout[0][j];
						}
					}
					else{
						kkkk=0;
						kkkk_t=0;
						ox_mode = 1;
					}					
				}
				else{
					kkkk=0;
					kkkk_t=0;
					
				}
			}
			
			if(ox_mode == 1){                // ((ox_mode==1)&&(myQueue.count == 0)&&(time_set_que.count == 0)&&(kkk==0)&&(kkkk==0)){   // start of compare mode
				mismatch = 0;
				for(int t = 0; t < passcode_que.data[1]; t++){
					if(passcode_que.data[t+2] != my_passcode_que.data[t+1]){
						mismatch++;
					}
				}
				if((ox_t<=3)&&(ox_t>=1)) {
					if(mismatch>=1){
						if(fail_3<2){
							GPIOC->ODR = font_X[0][j];
							
							enter = 0;
						}
						else if(fail_3 == 2){
							GPIOC->ODR = font_D[0][j];
							
							enter = 0;
						}
					}
					else{
						GPIOC->ODR = font_O[0][j];
						fail_3 = 0;
						
						enter = 0;
					}
				}
				else {
					enter = 0;
				}
			}
			
		
			j++;
			row <<= 1;

			if (row == 0x100) {
					row = 1;
					j = 0;
			}
			TIM1->SR &= ~(1 << 0); // clear UIF
    }
}


void TIM2_IRQHandler (void){  //0.5sec
    if(TIM2->SR & 0x0001) {
			q++;
			tt++;
			if(q==10000) q=0;
			if(q%4==0){
        float analog_voltage = (float)ADCConverted * (3.3f / 4096.0f);
        temperature = ((analog_voltage * 100.0f) - 50.0f)*10;
				temperature = abs(temperature);
				t_10 = (int)temperature/100;
				t_1 = (int)(temperature - t_10*100)/10;
				t_1_1 = (temperature - 100*t_10 - t_1*10);
			}
			
			if((data_r==12)&&(key_col != 0x0f)){
				pass_set_time++;
			}
			else if((data_r==12)&&(key_col == 0x0f)){
				pass_set_time = 0;
			}	
			
      TIM2->SR &= ~(1 << 0); // clear UIF
    }
}

void TIM3_IRQHandler(void){ //1sec
    if (TIM3->SR & 0x0001){
        i++;
        if (i == 10) {
            i = 0;
            k++;
            if (k == 6) {
                k = 0;
                m++;
                if (m == 10) {
                    m = 0;
                    n++;
                    if (n == 6) {
                        n = 0;
                        a++;
												c++;
                        if (a == 10) {
                            a = 0;
                        }
                        if(c==10) b=1;
												if(c==20) b=2;
												if(c==24){
													b=0;
													c=0;
													a=0;
												}											
                    }
                }
            }
        }
			if((myQueue.count >= 1|p==9)){
        p++;
        if(p == 10){
            p = 0;
        }
			}
			if(time_set_que.count >= 1|r==8){
        r++;
        if(r == 9){
            r = 0;
        }
			}
			
			if((tt>4)&&(time_set_que.count>=2)){
				tt_b++;
				if(tt_b>2){
						dequeue(&time_set_que);
						dequeue(&time_set_que);
						dequeue(&time_set_que);
						dequeue(&time_set_que);
						dequeue(&time_set_que);
						dequeue(&time_set_que);
						dequeue(&time_set_que);
						tt_b = 0;
				}			
			}	
			
			if(kkk==1){
				kkk_t++;
				if(kkk_t == 10000) kkk_t = 0;
				if(passcode_que.data[passcode_que.rear]!=12){
					kkk_tt++;
					if(kkk_tt == 10000) kkk_tt = 0;
				}
			}
			
			if((kkk==1)&&(kkk_tt >= 3)){    //passcode setup time out
				kkk=0;
				kkk_t=0;
				kkk_tt=0;
			}
			
			if(kkkk==1){
				kkkk_t++;
				if(kkkk_t == 10000) kkkk_t = 0;
				if(enter != 1){
					kkkk_tt++;
					if(kkkk_tt == 10000) kkkk_tt = 0;
				}
			}

			if((kkkk==1)&&(kkkk_tt >= 3)){    //passcode guess time out
				kkkk=0;
				kkkk_t=0;
				kkkk_tt=0;
			}

			if(ox_mode == 1){
				ox_t++;
				if(ox_t == 4) {
					ox_mode = 0;
					ox_t = 0;
					if(mismatch>=1) fail_3++;
				}
					
			}

        TIM3->SR &= ~(1 << 0); // clear UIF
    }
}

void TIM4_IRQHandler (void){
			if((TIM4->SR & 0x0001) != 0) {
				GPIOB->BSRR = (~(key_row << 12) & 0xF000) | (key_row << 28);   //key row scan
				for (int t = 0; t < 1000; t++) { ; }           //bouncing elimination
				key_col = GPIOB->IDR;
				key_col = (key_col >> 8) & 0x0F;
				col_scan = 0x01;
				for (int t = 0; t < 4; t++) {      //key column scan
					if ((key_col & col_scan) == 0){
						data = key_index ;  //dot matrix output								
					}						
					col_scan = col_scan << 1;
					key_index = key_index + 1;
				}
				key_row = key_row << 1;
				if (key_row == 0x100) {
					key_row = 0x01;
					key_index = 0;
				}
				
				if(key_col != 0x0f){
						if((data_r != 15) && (time_set_que.count == 0) && (kkk == 0) && (kkkk == 0)){
							enqueue(&myQueue, data_r);
							for (int t = 0; t < 600000; t++) { ; }  //debouncing 
							if(data_r == 12) dequeue(&myQueue);
						}
						else if((data_r == 15 | time_set_que.count != 0) && (kkk == 0) && (kkkk == 0)){   //(data == 15 or time_set_que.count != 0)
							enqueue(&time_set_que, data_r);
							for (int t = 0; t < 600000; t++) { ; }  //debouncing
							tt = 0;
							if(data_r == 12) dequeue(&time_set_que);
						}
						if(data_r == 12){      //passcode setup start
							if((pass_set_time>=6) && (fail_3 != 3)){  
								initQueue(&passcode_que);								
								enqueue(&passcode_que, data_r);
								kkk = 1;
								kkkk = 0;
								kkk_tt=0;
								for (int t = 0; t < 600000; t++) { ; }  //debouncing
								pass_set_time = 0;
							}
							else if((passcode_que.count>=4)&& (fail_3 != 3)&& (my_passcode_que.rear == -1)){   //kkkk set when passcode is set
								enqueue(&my_passcode_que, data_r); //passcode guess start
								kkkk = 1;
								kkkk_tt=0;
							}	
						}
					}						
				
				if(passcode_que.count==1){
					if((data_r>=0x4) && (data_r<=0x8) && (key_col != 0x0f)){
						enqueue(&passcode_que, data_r);
						for (int t = 0; t < 600000; t++) { ; }  //debouncing
						kkk_tt = 0;									
					}
				}
				else if(passcode_que.count >=2){
					if((data_r>=0x0) && (data_r<=0x9) && (key_col != 0x0f)){
						if((passcode_que.data[1]+1) > passcode_que.rear){
							enqueue(&passcode_que, data_r);
							for (int t = 0; t < 600000; t++) { ; }  //debouncing
							kkk_tt = 0;																			
						}
					}
					else if(key_col != 0x0f){
						if((passcode_que.data[1]+1) == passcode_que.rear){  //last data of passcode_que for # to save the passcode
							enqueue(&passcode_que, data_r);
							for (int t = 0; t < 600000; t++) { ; }  //debouncing
							kkk_tt = 0;
							if(data_r == 12) initQueue(&my_passcode_que);
						}
					}
				}

			  if(my_passcode_que.count>=1){
					if((data_r>=0x0) && (data_r<=0x9) && (key_col != 0x0f)){
						enqueue(&my_passcode_que, data_r);
						for (int t = 0; t < 600000; t++) { ; }  //debouncing
						kkkk_tt = 0;
					}
					if((data_r==12)&& (key_col != 0x0f)&&(my_passcode_que.count>=5) && (my_passcode_que.data[my_passcode_que.rear]!=12) && (my_passcode_que.rear != -1)) {
						enqueue(&my_passcode_que, data_r);
						for (int t = 0; t < 600000; t++) { ; }  //debouncing
						enter=1;
						kkkk_tt=0;
					}
				}
																									
			TIM4->SR &= ~(1<<0); //clear UIF
		}
	}

void USART1_IRQHandler (void){
	if (USART1->SR & 0x20){
		data_r = USART1->DR;
		enqueue(&r_data_que, data_r);
	}
}
