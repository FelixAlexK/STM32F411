#include "stm32f4xx_hal.h"


//=============================================================================//
// Die init()-Methode wird vor der main-Loop aufgerufen

void init()
{
  //---------------------------------------------------------------------------//
  // Initialisierung der Clocks
    RCC->CR |= (1 << 16); //Bit16: Enable RCC clock
    RCC->AHB1ENR |= (1 << 0) | (1 << 2); //Bit0: Enable clocks for GPIOA, Bit2: Enable clock for GPIOC
    
  //---------------------------------------------------------------------------//
  // Konfigurierung des Timers 4 f?r die Blinkgeschwindigkeit
    RCC->APB1ENR |= (1 << 2); // Enable the clock for TIM4
    TIM4->PSC = 8399; //prescaler 
    TIM4->ARR = 999; //auto-reload
    TIM4->DIER |= (1 << 0); //Enable the update interrupt
  //---------------------------------------------------------------------------//
  // Konfigurierung des Entprellungstimers (Timer 3)
    RCC->APB1ENR |= (1 << 1); //Bit1: Enable TIM3
    TIM3->PSC = 83; // prescaler(PSC)
    TIM3->EGR = (1 << 0); //set update generation (UG)
    TIM3->ARR = 999; // auto-reload(ARR)
    TIM3->CNT = 0; 
    TIM3->DIER = (1 << 0); //Enable the update interrupt
  //---------------------------------------------------------------------------//
  // Systemkonfigurationskontroller (SYSCFG)
    RCC->APB2ENR |= (1 << 14);  // Enable clock for SYSCFG
    
  //---------------------------------------------------------------------------//
  // Port-Interrupt-Konfigurierung f?r den Button (EXTI) (Port C Pin 13)
    SYSCFG->EXTICR[3] |= (1 << 5); // Connect EXTI13 to button pin 13
    GPIOC->MODER |= (0 << 26);  //Set the MODER13 bit to 0 (input mode)
    GPIOC->PUPDR |= (1 << 26);// Set the PUPDR13 bits to 1 (pull-up)
    EXTI->FTSR |= (1 << 13); // Falling edge trigger
    EXTI->IMR |= (1 << 13); // Enable the port interrupt for Pin 13 on Port C
  //---------------------------------------------------------------------------//
  // LED-Konfigurierung (Port A Pin 5)
    GPIOA->MODER |= (1 << 10);  //Bits are Output
            
  //---------------------------------------------------------------------------//
  // Interruptfreigabe
    NVIC_EnableIRQ(EXTI15_10_IRQn); // enable interrupt for EXTI13
    NVIC_EnableIRQ(TIM3_IRQn); //enable interrupt for TIM3
    NVIC_SetPriority(TIM3_IRQn, 1); //set priority of TIM3 interrupt
    NVIC_EnableIRQ(TIM4_IRQn);//enable interrupt for TIM$
    NVIC_SetPriority(TIM4_IRQn, 3);//set priority of TIM4 interrupt
  //---------------------------------------------------------------------------//
  // Globale Interruptfreigabe
    __enable_irq(); //enable global interrupt
}




//=============================================================================//
// Die main-Methode

int main(void)
{
  init();
  
  while(1)
  {
    
  }
}

//=============================================================================//
// Interrupt-Service-Routine f?r den Button

void EXTI15_10_IRQHandler(void)
{
  
  
  if (EXTI->PR & (1 << 13))
  {
    // Button press detected
    if (!(GPIOC->IDR & (1 << 13)))
    {
      // Start the debounce timer
      TIM3->CNT = 0; //set counter to 0
      TIM3->CR1 |= (1 << 0); //enable timer
    }
    // Clear the interrupt flag
    EXTI->PR |= (1 << 13);
  }
  
  
    
 }
   

//=============================================================================//
// Interrupt-Service-Routine f?r den Blinktimer (Timer 4)

void TIM4_IRQHandler(void)
{
  if (TIM4->SR & (1 << 0))
  {
    
    // Toggle the LED
    GPIOA->ODR ^= (1 << 5);
    
    // Clear the update interrupt flag
    TIM4->SR &= (0 << 0);
  }
  
  
}

//=============================================================================//
// Interrupt-Service-Routine f?r den Entprellungstimer (Timer 3)

void TIM3_IRQHandler(void)
{
  
  if (TIM3->SR & (1 << 0))
  {
  
   // Debounce time elapsed
    
      // Button is pressed
      // Register button press event
      TIM4->CR1 ^= (1 << 0); //toggle bit to disable or enable timer
      
    
    // Stop the debounce timer
    TIM3->CR1 &= ~(1 << 0);
    // Clear the interrupt flag
    TIM3->SR &= ~(1 << 0);
  }
}