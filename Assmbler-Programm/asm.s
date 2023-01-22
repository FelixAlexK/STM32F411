        PUBLIC  __iar_program_start
        PUBLIC  __vector_table

        SECTION .text:CODE:REORDER(1)
        
        ;; Keep vector table even if it's not referenced
        REQUIRE __vector_table
        
        THUMB
        
__iar_program_start
        
main    
        BL      Init
        
loop
        B       loop
        
Init
   
/*Clock-Konfiguration*/
  
        ldr     r0, =0x40023800                 ; RCC base address
        ldr     r1, [r0, #0x00]                 ; RCC->CR
        ldr     r2, =0x00010000                 ; (1 << 16)
        orr     r1, r1, r2                      ; RCC->CR |= (1 << 16)
        str     r1, [r0, #0x00]                 ; store RCC->CR

        ldr     r3, [r0, #0x30]                 ; RCC->AHB1ENR
        ldr     r4, =0x00000005                 ; (1 << 0) | (1 << 2)
        orr     r3, r3, r4                      ; RCC->AHB1ENR |= (1 << 0) | (1 << 2)
        str     r3, [r0, #0x30]                 ; store RCC->AHB1ENR
/*********************************************************************************************************************************************/

/*Timer-Konfiguration für Timer 4 und Timer 3*/
        //TIM4
        ldr     r0, =0x40023840                 ; RCC base address
        ldr     r1, [r0, #0x38]                 ; RCC->APB1ENR
        ldr     r2, =0x00000004                 ; (1 << 2)
        orr     r1, r1, r2                      ; RCC->APB1ENR |= (1 << 2)
        str     r1, [r0, #0x38]                 ; store RCC->APB1ENR

        ldr     r0, =0x40006800                 ; TIM4 base address
        ldr     r3, =8399                       ; prescaler value
        str     r3, [r0, #0x28]                 ; TIM4->PSC = 8399

        ldr     r4, =999                        ; auto-reload value
        str     r4, [r0, #0x2C]                 ; TIM4->ARR = 999

        ldr     r5, [r0, #0x0C]                 ; TIM4->DIER
        ldr     r6, =0x00000001                 ; (1 << 0)
        orr     r5, r5, r6                      ; TIM4->DIER |= (1 << 0)
        str     r5, [r0, #0x0C]                 ; store TIM4->DIER

        //TIM3
        ldr     r0, =0x40023840                 ; RCC base address
        ldr     r1, [r0, #0x38]                 ; RCC->APB1ENR
        ldr     r2, =0x00000002                 ; (1 << 1)
        orr     r1, r1, r2                      ; RCC->APB1ENR |= (1 << 1)
        str     r1, [r0, #0x38]                 ; store RCC->APB1ENR

        ldr     r0, =0x40000400                 ; TIM3 base address
        ldr     r3, =83                         ; prescaler value
        str     r3, [r0, #0x28]                 ; TIM3->PSC = 83

        ldr     r4, =1                          ; (1 << 0)
        str     r4, [r0, #0x14]                 ; TIM3->EGR = (1 << 0)

        ldr     r5, =999                        ; auto-reload value
        str     r5, [r0, #0x2C]                 ; TIM3->ARR = 999
  
        mov     r6, #0                          ; counter value
        str     r6, [r0, #0x24]                 ; TIM3->CNT = 0

        ldr     r7, [r0, #0x0C]                 ; TIM3->DIER
        ldr     r8, =0x00000001                 ; (1 << 0)
        orr     r7, r7, r8                      ; TIM3->DIER |= (1 << 0)
        str     r7, [r0, #0x0C]                 ; store TIM3->DIER


/***********************************************************************************************************************************************/

/*Systemkonfiguration*/

        ldr     r0, =0x40023844                 ; RCC base address
        ldr     r1, [r0, #0x0C]                 ; RCC->APB2ENR
        ldr     r2, =0x00004000                 ; (1 << 14)
        orr     r1, r1, r2                      ; RCC->APB2ENR |= (1 << 14)
        str     r1, [r0, #0x0C]                 ; store RCC->APB2ENR

/***********************************************************************************************************************************************/       
        
/*Port-Interrupt-Konfiguration*/

        ldr     r0, =0x40013800                 ; SYSCFG base address
        ldr     r1, [r0, #0x14]                 ; SYSCFG->EXTICR[3]
        ldr     r2, =0x00000020                 ; (1 << 5)
        orr     r1, r1, r2                      ; SYSCFG->EXTICR[3] |= (1 << 5)
        str     r1, [r0, #0x14]                 ; store SYSCFG->EXTICR[3]

        ldr     r0, =0x40011000                 ; GPIOC base address
        ldr     r3, [r0, #0x00]                 ; GPIOC->MODER
        ldr     r4, =0x00000000                 ; (0 << 26)
        bic     r3, r3, r4                      ; GPIOC->MODER &= ~(0 << 26)
        str     r3, [r0, #0x00]                 ; store GPIOC->MODER

        ldr     r5, [r0, #0x0C]                 ; GPIOC->PUPDR
        ldr     r6, =0x04000000                 ; (1 << 26)
        orr     r5, r5, r6                      ; GPIOC->PUPDR |= (1 << 26)
        str     r5, [r0, #0x0C]                 ; store GPIOC->PUPDR

        ldr     r0, =0x40013C0C                 ; EXTI base address
        ldr     r7, [r0, #0x04]                 ; EXTI->FTSR
        ldr     r8, =0x2000                     ; (1 << 13)
        orr     r7, r7, r8                      ; EXTI->FTSR |= (1 << 13)
        str     r7, [r0, #0x04]                 ; store EXTI->FTSR

        ldr     r9, [r0, #0x00]                 ; EXTI->IMR
        ldr     r10, =0x2000                    ; (1 << 13)
        orr     r9, r9, r10                     ; EXTI->IMR |= (1 << 13)
        str     r9, [r0, #0x00]                 ; store EXTI->IMR

/************************************************************************************************************************************************/
        
/*LED-Konfiguration*/

        ldr     r0, =0x40020000                 ; GPIOA base address
        ldr     r1, [r0, #0x00]                 ; GPIOA->MODER
        ldr     r2, =0x00000400                 ; (1 << 10)
        orr     r1, r1, r2                      ; GPIOA->MODER |= (1 << 10)
        str     r1, [r0, #0x00]                 ; store GPIOA->MODER

/************************************************************************************************************************************************/
/*Da sich der NVIC im Prozessor befindet, muss hier in das Technical-Reference-Manual vom Prozessor, also dem Cortex-M4 geschaut werden,
  wie man die jeweiligen Leitungen aktiviert, bzw. wo man das machen kann. Auf Seite 208 in Tabelle 45 sieht man die NVIC-Register aufgelistet.
  Der benötigte Adressbereich steht in der ersten Spalte. Für das Interrupt set-enable register x (NVIC_ISERx), welcher in der ersten Zeile ist, steht
  ein Adressbereich von 0xE000E100 bis 0xE000E11F. Wie beim vorherigen Code auch laden wir die Startadrese in ein Register, hier das R0-Register, und
  addieren den Offset drauf. Da das NVIC_ISERx bis zu 8 Register besitzt, müssen wir den Offset, selber ausrechnen. Die Formel steht auf Seite 210 und
  ist folgende: 0x100 + 0x04 * x (x = 0 bis 7). Damit man weiß, was man für x einsetzen muss, braucht man wieder das Reference-Manual vom STM32.
  Auf Seite 201 beginnend, sieht man eine Tabelle mit allen möglichen Interrupts, die ausgelöst werden können, die sogenannte Vector-Table. Beginnen wir
  mit dem Timer 3, den wir oben für Interrupts konfiguriert haben. Dem Vector-Table können wir entnehmen, dass die Position vom Timer 3 29 ist.
  Wenn wir uns mit dieser Zahl wieder Seite 210 vom Technical-Reference-Manual des Cortex-M4-Prozessors wenden, können wir ablesen, dass unser x hier 0
  sein muss. Denn 4 Zeilen nach der Überschrift steht: NVIC_ISER0 bits 0 to 31 are for interrupt 0 to 31, respectively. Da wir uns mit 29 im Bereich
  zwischen 0 und 31 befinden, müssen wir für unser x 0 wählen. Setzen wir nun das x in die Formel ein, ergibt sich ein Adress-Offset von 0x100.
  Wichtig zu beachten ist, da der Speicherbereich bei 0xE000E100 anfängt und das NVIC_ISER0 das erste Register ist, befindet sich der Offset bereits in dieser
  Startadresse. Das heißt der Offset von 0x100 wurde bereits zu 0xE000E100 addiert. Es ist leider etwas anders als im Reference-Manual vom STM32. Man kann
  sich aber vorstellen, dass man den berechneten Offset zu 0xE000E000 addiert.
  Laden wir nun dieses NVIC_ISER0 in ein Register, hier das R1-Register, müssen wir nur noch das Bit an Stelle 29 auf 1 setzen und es wieder zurückschreiben.*/

/*NVIC-Interrupt-Konfiguration*/
    /*NVIC_ISERx*/
      /*Erlaube am NVIC Timer3- und Timer4-Interrupts*/
        LDR     R0, =0xE000E100
        LDR     R1, [R0]
        ORR     R1, #0x60000000
        STR     R1, [R0]
        
      /*Erlaube am NVIC Port-Interrupts für die GPIO-Pins von 10 bis einschließlich 15*/
        LDR     R0, =0xE000E104
        LDR     R1, [R0]
        ORR     R1, #0x00000100
        STR     R1, [R0]
        
/************************************************************************************************************************************************/
    
/*Global Interrupt-Freigabe*/    
        CPSIE   i
        
/*Springe zurück zur vorherigen Methode*/

        BX      LR
        
/************************************************************************************************************************************************
*************************************************************************************************************************************************/

/*Interrupt-Handler für den Port-Interrupt*/
EXTI15_10_ISR
        ldr     r0, =0x40013C0C                 ; EXTI base address
        ldr     r1, [r0, #0x08]                 ; EXTI->PR
        ldr     r2, =0x2000                     ; (1 << 13)
        tst     r1, r2                          ; check EXTI->PR & (1 << 13)
        bne     button_press                    ; branch if button press detected

        bx      lr                              ; return if no button press detected

    button_press:
        ldr     r0, =0x40011000                 ; GPIOC base address
        ldr     r3, [r0, #0x10]                 ; GPIOC->IDR
        ldr     r4, =0x2000                     ; (1 << 13)
        tst     r3, r4                          ; check GPIOC->IDR & (1 << 13)
        beq     debounce_timer                  ; branch if button is pressed
        bx      lr                              ; return if button is not pressed

    debounce_timer:
        ldr     r0, =0x40000400                 ; TIM3 base address
        mov     r5, #0                          ; set counter to 0
        str     r5, [r0, #0x24]                 ; TIM3->CNT = 0

        ldr     r6, [r0, #0x00]                 ; TIM3->CR1
        ldr     r7, =0x0001                     ; (1 << 0)
        orr     r6, r6, r7                      ; TIM3->CR1 |= (1 << 0)
        str     r6, [r0, #0x00]                 ; store TIM3->CR1

        ldr     r8, [r0, #0x08]                 ; EXTI->PR
        ldr     r9, =0x2000                     ; (1 << 13)
        orr     r8, r8, r9                      ; EXTI->PR |= (1 << 13)
        str     r8, [r0, #0x08]                 ; store EXTI->PR
  
                             
        bx      lr                              ; return
 
/************************************************************************************************************************************************
*************************************************************************************************************************************************/
 
/*Interrupt-Handler für Timer 4*/
TIM4_ISR

        ldr     r0, =0x40000800                 ; TIM4 base address
        ldr     r1, [r0, #0x10]                 ; TIM4->SR
        ldr     r2, =0x0001                     ; (1 << 0)
        tst     r1, r2                          ; check TIM4->SR & (1 << 0)
        bne     toggle_led                      ; branch if update interrupt flag is set

        bx      lr                              ; return if update interrupt flag is not set

    toggle_led:
        ldr     r0, =0x40020000                 ; GPIOA base address
        ldr     r3, [r0, #0x14]                 ; GPIOA->ODR
        ldr     r4, =0x00000020                 ; (1 << 5)
        eor     r3, r3, r4                      ; GPIOA->ODR ^= (1 << 5)
        str     r3, [r0, #0x14]                 ; store GPIOA->ODR

        ldr     r5, [r0, #0x10]                 ; TIM4->SR
        ldr     r6, =0x0000                     ; (0 << 0)
        and     r5, r5, r6                      ; TIM4->SR &= (0 << 0)
        str     r5, [r0, #0x10]                 ; store TIM4->SR
 
 
        bx     lr                               ; return

/************************************************************************************************************************************************
*************************************************************************************************************************************************/

/*Interrupt-Handler für Timer 3*/
TIM3_ISR
        ldr     r0, =0x40000400                 ; TIM3 base address
        ldr     r1, [r0, #0x10]                 ; TIM3->SR
        ldr     r2, =0x0001                     ; (1 << 0)
        tst     r1, r2                          ; check TIM3->SR & (1 << 0)
        bne     debounce_elapsed                ; branch if debounce time elapsed
        bx      lr                              ; return if debounce time not elapsed

    debounce_elapsed:
        ldr     r0, =0x40000800                 ; TIM4 base address
        ldr     r3, [r0, #0x00]                 ; TIM4->CR1
        ldr     r4, =0x0001                     ; (1 << 0)
        eor     r3, r3, r4                      ; TIM4->CR1 ^= (1 << 0)
        str     r3, [r0, #0x00]                 ; store TIM4->CR1

        ldr     r5, [r0, #0x00]                 ; TIM3->CR1
        ldr     r6, =0x0001                     ; ~(1 << 0)
        bic     r5, r5, r6                      ; TIM3->CR1 &= ~(1 << 0)
        str     r5, [r0, #0x00]                 ; store TIM3->CR1

        ldr     r7, [r0, #0x10]                 ; TIM3->SR
        ldr     r8, =0x0001                     ; ~(1 << 0)
        bic     r7, r7, r8                      ; TIM3->SR &= ~(1 << 0)
        str     r7, [r0, #0x10]                 ; store TIM3->SR
        
                               
        bx      lr                              ; return
        
        


        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)
        SECTION .intvec:CODE:NOROOT(2)
        
        DATA

__vector_table
        DCD     sfe(CSTACK)
        DCD     __iar_program_start

        DCD     NMI_Handler
        DCD     HardFault_Handler
        DCD     MemManage_Handler
        DCD     BusFault_Handler
        DCD     UsageFault_Handler
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     SVC_Handler
        DCD     DebugMon_Handler
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler
        
         ; External Interrupts
        DCD     WWDG_IRQHandler                   ; Window WatchDog
        DCD     PVD_IRQHandler                    ; PVD through EXTI Line detection
        DCD     TAMP_STAMP_IRQHandler             ; Tamper and TimeStamps through the EXTI line
        DCD     RTC_WKUP_IRQHandler               ; RTC Wakeup through the EXTI line
        DCD     FLASH_IRQHandler                  ; FLASH
        DCD     RCC_IRQHandler                    ; RCC
        DCD     EXTI0_IRQHandler                  ; EXTI Line0
        DCD     EXTI1_IRQHandler                  ; EXTI Line1
        DCD     EXTI2_IRQHandler                  ; EXTI Line2
        DCD     EXTI3_IRQHandler                  ; EXTI Line3
        DCD     EXTI4_IRQHandler                  ; EXTI Line4
        DCD     DMA1_Stream0_IRQHandler           ; DMA1 Stream 0
        DCD     DMA1_Stream1_IRQHandler           ; DMA1 Stream 1
        DCD     DMA1_Stream2_IRQHandler           ; DMA1 Stream 2
        DCD     DMA1_Stream3_IRQHandler           ; DMA1 Stream 3
        DCD     DMA1_Stream4_IRQHandler           ; DMA1 Stream 4
        DCD     DMA1_Stream5_IRQHandler           ; DMA1 Stream 5
        DCD     DMA1_Stream6_IRQHandler           ; DMA1 Stream 6
        DCD     ADC_IRQHandler                    ; ADC1
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     EXTI9_5_IRQHandler                ; External Line[9:5]s
        DCD     TIM1_BRK_TIM9_IRQHandler          ; TIM1 Break and TIM9
        DCD     TIM1_UP_TIM10_IRQHandler          ; TIM1 Update and TIM10
        DCD     TIM1_TRG_COM_TIM11_IRQHandler     ; TIM1 Trigger and Commutation and TIM11
        DCD     TIM1_CC_IRQHandler                ; TIM1 Capture Compare
        DCD     TIM2_IRQHandler                   ; TIM2
        DCD     TIM3_IRQHandler                   ; TIM3
        DCD     TIM4_IRQHandler                   ; TIM4
        DCD     I2C1_EV_IRQHandler                ; I2C1 Event
        DCD     I2C1_ER_IRQHandler                ; I2C1 Error
        DCD     I2C2_EV_IRQHandler                ; I2C2 Event
        DCD     I2C2_ER_IRQHandler                ; I2C2 Error
        DCD     SPI1_IRQHandler                   ; SPI1
        DCD     SPI2_IRQHandler                   ; SPI2
        DCD     USART1_IRQHandler                 ; USART1
        DCD     USART2_IRQHandler                 ; USART2
        DCD     0                                 ; Reserved
        DCD     EXTI15_10_IRQHandler              ; External Line[15:10]s
        DCD     RTC_Alarm_IRQHandler              ; RTC Alarm (A and B) through EXTI Line
        DCD     OTG_FS_WKUP_IRQHandler            ; USB OTG FS Wakeup through EXTI line
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     DMA1_Stream7_IRQHandler           ; DMA1 Stream7
        DCD     0                                 ; Reserved
        DCD     SDIO_IRQHandler                   ; SDIO
        DCD     TIM5_IRQHandler                   ; TIM5
        DCD     SPI3_IRQHandler                   ; SPI3
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     DMA2_Stream0_IRQHandler           ; DMA2 Stream 0
        DCD     DMA2_Stream1_IRQHandler           ; DMA2 Stream 1
        DCD     DMA2_Stream2_IRQHandler           ; DMA2 Stream 2
        DCD     DMA2_Stream3_IRQHandler           ; DMA2 Stream 3
        DCD     DMA2_Stream4_IRQHandler           ; DMA2 Stream 4
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     OTG_FS_IRQHandler                 ; USB OTG FS
        DCD     DMA2_Stream5_IRQHandler           ; DMA2 Stream 5
        DCD     DMA2_Stream6_IRQHandler           ; DMA2 Stream 6
        DCD     DMA2_Stream7_IRQHandler           ; DMA2 Stream 7
        DCD     USART6_IRQHandler                 ; USART6
        DCD     I2C3_EV_IRQHandler                ; I2C3 event
        DCD     I2C3_ER_IRQHandler                ; I2C3 error
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     FPU_IRQHandler                    ; FPU
        DCD     0                                 ; Reserved
        DCD     0                                 ; Reserved
        DCD     SPI4_IRQHandler                   ; SPI4
        DCD     SPI5_IRQHandler                   ; SPI5

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;

        PUBWEAK NMI_Handler
        PUBWEAK HardFault_Handler
        PUBWEAK MemManage_Handler
        PUBWEAK BusFault_Handler
        PUBWEAK UsageFault_Handler
        PUBWEAK SVC_Handler
        PUBWEAK DebugMon_Handler
        PUBWEAK PendSV_Handler
        PUBWEAK SysTick_Handler

        SECTION .text:CODE:REORDER:NOROOT(1)
        THUMB

NMI_Handler
HardFault_Handler
MemManage_Handler
BusFault_Handler
UsageFault_Handler
SVC_Handler
DebugMon_Handler
PendSV_Handler
SysTick_Handler
Default_Handler
__default_handler
        CALL_GRAPH_ROOT __default_handler, "interrupt"
        NOCALL __default_handler
        B __default_handler
        
        PUBWEAK WWDG_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
WWDG_IRQHandler  
        B WWDG_IRQHandler

        PUBWEAK PVD_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PVD_IRQHandler  
        B PVD_IRQHandler

        PUBWEAK TAMP_STAMP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TAMP_STAMP_IRQHandler  
        B TAMP_STAMP_IRQHandler

        PUBWEAK RTC_WKUP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RTC_WKUP_IRQHandler  
        B RTC_WKUP_IRQHandler

        PUBWEAK FLASH_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
FLASH_IRQHandler  
        B FLASH_IRQHandler

        PUBWEAK RCC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RCC_IRQHandler  
        B RCC_IRQHandler

        PUBWEAK EXTI0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXTI0_IRQHandler  
        B EXTI0_IRQHandler

        PUBWEAK EXTI1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXTI1_IRQHandler  
        B EXTI1_IRQHandler

        PUBWEAK EXTI2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXTI2_IRQHandler  
        B EXTI2_IRQHandler

        PUBWEAK EXTI3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXTI3_IRQHandler
        B EXTI3_IRQHandler

        PUBWEAK EXTI4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXTI4_IRQHandler  
        B EXTI4_IRQHandler

        PUBWEAK DMA1_Stream0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA1_Stream0_IRQHandler  
        B DMA1_Stream0_IRQHandler

        PUBWEAK DMA1_Stream1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA1_Stream1_IRQHandler  
        B DMA1_Stream1_IRQHandler

        PUBWEAK DMA1_Stream2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA1_Stream2_IRQHandler  
        B DMA1_Stream2_IRQHandler

        PUBWEAK DMA1_Stream3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA1_Stream3_IRQHandler  
        B DMA1_Stream3_IRQHandler

        PUBWEAK DMA1_Stream4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA1_Stream4_IRQHandler  
        B DMA1_Stream4_IRQHandler

        PUBWEAK DMA1_Stream5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA1_Stream5_IRQHandler  
        B DMA1_Stream5_IRQHandler

        PUBWEAK DMA1_Stream6_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA1_Stream6_IRQHandler  
        B DMA1_Stream6_IRQHandler

        PUBWEAK ADC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC_IRQHandler  
        B ADC_IRQHandler

        PUBWEAK EXTI9_5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXTI9_5_IRQHandler  
        B EXTI9_5_IRQHandler

        PUBWEAK TIM1_BRK_TIM9_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM1_BRK_TIM9_IRQHandler  
        B TIM1_BRK_TIM9_IRQHandler

        PUBWEAK TIM1_UP_TIM10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM1_UP_TIM10_IRQHandler  
        B TIM1_UP_TIM10_IRQHandler

        PUBWEAK TIM1_TRG_COM_TIM11_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM1_TRG_COM_TIM11_IRQHandler  
        B TIM1_TRG_COM_TIM11_IRQHandler
        
        PUBWEAK TIM1_CC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM1_CC_IRQHandler  
        B TIM1_CC_IRQHandler

        PUBWEAK TIM2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM2_IRQHandler  
        B TIM2_IRQHandler

        PUBWEAK TIM3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM3_IRQHandler  
        B TIM3_ISR

        PUBWEAK TIM4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM4_IRQHandler  
        B TIM4_ISR

        PUBWEAK I2C1_EV_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C1_EV_IRQHandler  
        B I2C1_EV_IRQHandler

        PUBWEAK I2C1_ER_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C1_ER_IRQHandler  
        B I2C1_ER_IRQHandler

        PUBWEAK I2C2_EV_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C2_EV_IRQHandler  
        B I2C2_EV_IRQHandler

        PUBWEAK I2C2_ER_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C2_ER_IRQHandler  
        B I2C2_ER_IRQHandler

        PUBWEAK SPI1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI1_IRQHandler  
        B SPI1_IRQHandler

        PUBWEAK SPI2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI2_IRQHandler  
        B SPI2_IRQHandler

        PUBWEAK USART1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
USART1_IRQHandler  
        B USART1_IRQHandler

        PUBWEAK USART2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
USART2_IRQHandler  
        B USART2_IRQHandler

        PUBWEAK EXTI15_10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXTI15_10_IRQHandler  
        B EXTI15_10_ISR

        PUBWEAK RTC_Alarm_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RTC_Alarm_IRQHandler  
        B RTC_Alarm_IRQHandler

        PUBWEAK OTG_FS_WKUP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OTG_FS_WKUP_IRQHandler  
        B OTG_FS_WKUP_IRQHandler

        PUBWEAK DMA1_Stream7_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA1_Stream7_IRQHandler  
        B DMA1_Stream7_IRQHandler

        PUBWEAK SDIO_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SDIO_IRQHandler  
        B SDIO_IRQHandler

        PUBWEAK TIM5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM5_IRQHandler  
        B TIM5_IRQHandler

        PUBWEAK SPI3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI3_IRQHandler  
        B SPI3_IRQHandler

        PUBWEAK DMA2_Stream0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA2_Stream0_IRQHandler  
        B DMA2_Stream0_IRQHandler

        PUBWEAK DMA2_Stream1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA2_Stream1_IRQHandler  
        B DMA2_Stream1_IRQHandler

        PUBWEAK DMA2_Stream2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA2_Stream2_IRQHandler  
        B DMA2_Stream2_IRQHandler

        PUBWEAK DMA2_Stream3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA2_Stream3_IRQHandler  
        B DMA2_Stream3_IRQHandler

        PUBWEAK DMA2_Stream4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA2_Stream4_IRQHandler  
        B DMA2_Stream4_IRQHandler

        PUBWEAK OTG_FS_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OTG_FS_IRQHandler  
        B OTG_FS_IRQHandler

        PUBWEAK DMA2_Stream5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA2_Stream5_IRQHandler  
        B DMA2_Stream5_IRQHandler

        PUBWEAK DMA2_Stream6_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA2_Stream6_IRQHandler  
        B DMA2_Stream6_IRQHandler

        PUBWEAK DMA2_Stream7_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA2_Stream7_IRQHandler  
        B DMA2_Stream7_IRQHandler

        PUBWEAK USART6_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
USART6_IRQHandler  
        B USART6_IRQHandler

        PUBWEAK I2C3_EV_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C3_EV_IRQHandler  
        B I2C3_EV_IRQHandler

        PUBWEAK I2C3_ER_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C3_ER_IRQHandler  
        B I2C3_ER_IRQHandler

        PUBWEAK FPU_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
FPU_IRQHandler  
        B FPU_IRQHandler

        PUBWEAK SPI4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI4_IRQHandler  
        B SPI4_IRQHandler
		
        PUBWEAK SPI5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI5_IRQHandler  
        B SPI5_IRQHandler

        END
