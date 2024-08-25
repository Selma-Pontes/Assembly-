; Ficheiro:  semaforos.S
; Descricao: Programa de realização do TP4-projeto  de
;            Arquitetura de Computadores.
; Autor:     Selma Pontes e Isabel Clemente (grupo-08)
; Nº:           51523     e   51828
; Data:      30-05-2024

; Definicao dos valores dos simbolos utilizados no programa
;
	.equ	CPSR_BIT_I, 0b010000          ; Mascara para o bit I do registo CPSR

	.equ	STACK_SIZE, 64                ; Dimensao do stack - 64 B

	; Definicoes do porto de entrada
	.equ	INPORT_ADDRESS, 0xFF80        ; Endereco do porto de entrada

	; Definicoes do porto de saida
	.equ	OUTPORT_ADDRESS, 0xFFC0       ; Endereco do porto de saida

	.equ	OUTPORT_INIT_VAL, 0           ; Valor inicial do porto de saida

	; Definicoes do circuito pTC
; *** Inicio de troco para completar ***
	.equ	PTC_ADDRESS,  0xFF40          ; Endereco do circuito pTC
; *** Fim de troco para completar ***

	.equ	PTC_TCR, 0                    ; Deslocamento do registo TCR do pTC
	.equ	PTC_TMR, 2                    ; Deslocamento do registo TMR do pTC
	.equ	PTC_TC,  4                    ; Deslocamento do registo TC do pTC
	.equ	PTC_TIR, 6                    ; Deslocamento do registo TIR do pTC

	.equ	PTC_CMD_START, 0              ; Comando para iniciar a contagem no pTC
	.equ	PTC_CMD_STOP, 1               ; Comando para parar a contagem no pTC

; *** Inicio de troco para completar ***
	.equ	SYSCLK_FREQ,0x09            ; Intervalo de contagem do circuito pTC (100msec)
  ;no simulador        na placa                   ; que suporta a implementação do sysclk
 ;módulo 9                 99
 ;frequencia 100Hz        1KHz
; *** Fim de troco para completar ***
	; Outras definições 
	
;Mascaras para os bits consoante o funcionamento do sistema	
	.equ PORT_IN_BTN_MASK_PEDESTRIAN, 0x01   ;botão
	.equ PORT_IN_BTN_MASK_CONFIG,     0x10  ;16 em dec

	;máscaras  para modo operação:
	.equ    PORT_OUT_LED_MASK_AMARELO_L1, 				0x03          
	.equ    PORT_OUT_LED_MASK_VERMELHO_L2, 			0x04
	  
	 ;máscaras para modo de transição 0->1 (PRESSED-BUTTON)  
	 .equ    PORT_OUT_LED_MASK_VERMELHO_L1 , 		0x01
	 .equ    PORT_OUT_LED_MASK_VERDE_L2,     		0x08
     .equ    PORT_OUT_LED_MASK_OPERATION_MODE_LED_L3,  				0x20 ; 32 em decimal    

	;máscaras para o modo configuração (JÁ existem declaradas acima):
;	.equ    AMARELO_LED_L1 ,3
;	.equ    VERDE_LED_L2 , 8
    .equ    PORT_OUT_LED_MASK_CONFIGURATION_MODE_LED_L3,  0x30 ;48 em hexadecimal  
       

; *** Inicio de troco para completar ***
	.equ	BLINK_TIME,5                ; 50% do periodo a  em periodos do relogio do pTC 
	.equ  	CROSSING_TIME_DEFAULT_INDEX, 0 ; Pre-definido 10 segundos       
    .equ    TIMES_MASK, 0xE0                                              
; *** Fim de troco para completar ***
   
; Seccao:    startup
; Descricao: Guarda o código de arranque do sistema
;
	.section startup
	b	_start
	ldr	pc, isr_addr
_start:
	ldr	sp, stack_top_addr
    mov r0, pc
    add lr, r0, #4
	ldr	pc, main_addr
    b   .

stack_top_addr:
	.word	stack_top
main_addr:
	.word	main
isr_addr:
	.word	isr

; Seccao:    text
; Descricao: Guarda o código do programa
;
	.text

; Rotina:    main
; Descricao: Configura as inicializações necessárias para piscar um LED.
; Entradas:  -
; Saidas:    -
; Efeitos:   Configura o porto de saída, inicializa o relógio do sistema (sysclk) e inicia um loop para piscar um LED.
main:
	mov	r0, #OUTPORT_INIT_VAL
	bl	outport_init
	mov	r0, #SYSCLK_FREQ   
	bl	sysclk_init

	; initializar timer dos leds dos semaforos
	bl semaforo_timer_init
	bl semaforo_led_estado_init
	bl crossing_timer_init

	mrs	r0, cpsr
	mov	r1, #CPSR_BIT_I
	orr	r0, r0, r1
	msr	cpsr, r0
main_loop:
	bl  semaforo_main_function
	bl  button_pedestrian_main_function
	bl  config_mode_main_function
	bl  crossing_time_cfg_main_function
	b	main_loop

; Rotina:    crossing_time_cfg_main_function
; Descricao: Funcao que executa no modo de configuracao e atualiza o crossing time
;verificação se estamos no estado de configuração , 
;se tiver lemos a entrada e configura-se o valor de crossing_time
crossing_time_cfg_main_function:
	push lr

	ldr r2,addr_semaforo_led_estado_2
	ldr r2, [r2]
	mov r0, #2 
	cmp r2, r0
	bne crossing_time_cfg_main_function_end
	; modo configuracao
	bl inport_read
	mov r1, #TIMES_MASK
	and r0, r0, r1
	lsr r0,r0,#5

	; em r0 tenho valor index do timer
	mov r1, #5
	cmp r0, r1
	blo crossing_time_cfg_main_function_valid_cfg
	b crossing_time_cfg_main_function_end

crossing_time_cfg_main_function_valid_cfg:
	ldr r1,crossing_time_index_addr_2
	str r0,[r1]

crossing_time_cfg_main_function_end:
	pop pc

crossing_time_index_addr_2:
	.word	crossing_time_index

; Rotina:    config_mode_main_function
; Descricao: Funcao que monitoriza o modo de configuracao

config_mode_main_function:
	push lr

	bl inport_read
	ldr r2,addr_semaforo_led_estado_2

	mov r1, #PORT_IN_BTN_MASK_CONFIG
	and r0, r0, r1
	cmp r0, r1
	bne config_mode_main_function_not_active

	ldr r0,[r2]
	mov r1, #2
	cmp r0, r1
	beq config_mode_main_function_end

	str r1,[r2]  
	bl semaforo_timer_init
	bl semaforo_led_config_init
	b config_mode_main_function_end

config_mode_main_function_not_active:
	ldr r0,[r2]
	mov r1, #2
	cmp r0, r1
	bne config_mode_main_function_end

	bl semaforo_timer_init
	bl semaforo_led_estado_init

config_mode_main_function_end:
	pop pc
; Rotina:    button_pressed
; Descricao: Funcao que executa quando o botao for pressionado

button_pressed:
	push lr
	push r0
	push r1

	ldr r1,addr_semaforo_led_estado_2
	ldr r0,[r1]

	mov r2, #0
	cmp r0, r2
	bne button_pressed_modo_cfg
	
	mov r2, #1
	str r2, [r1]

	b button_init_ports_timer

button_pressed_modo_cfg:
	mov r2, #1
	cmp r0, r2
	bne button_pressed_end

button_init_ports_timer:
	mov r0, #PORT_OUT_LED_MASK_VERMELHO_L1
	mov r1, #PORT_OUT_LED_MASK_VERDE_L2
	orr r0, r0, r1
	mov r1, #PORT_OUT_LED_MASK_OPERATION_MODE_LED_L3
	orr r0, r0, r1

	bl outport_write
	bl crossing_timer_start

button_pressed_end:
	pop r1
	pop r0
	pop pc

addr_semaforo_led_estado_2:
     .word semaforo_led_estado

; Rotina:    button_main_function
; Descricao: Main function do butao. Monitoriza o botao
;se for detetada uma transição de 0->1 chamamos o button_pressed

button_pedestrian_main_function:
	push lr
	bl inport_read
	; r0 temos o valor do porto
	mov r1, #PORT_IN_BTN_MASK_PEDESTRIAN
	and r0, r0, r1
	cmp r0, r1

	bne button_pedestrian_main_function_end
	mov r1, #0

	ldr r2, addr_btn_estado
	ldr r3, [r2]
	cmp r3, r1
	bne button_pedestrian_main_function_end

	bl button_pressed
	
button_pedestrian_main_function_end:

	ldr r1, addr_btn_estado
	str r0, [r1]
	pop pc

addr_btn_estado:
     .word btn_estado


; Rotina:   semaforo_set_estado_operacao
; Descricao: Funcao de controlo dos leds quando sistema esta em modo operacao
;se trigger a 1 , tem de se comutar o led ,
;a variavel 'semaforo_estado_anterior' indica para onde vou , se tiver a 0 vai para 1 ou vice-versa 
;para quando tem de se tem de fazer o led  piscar
semaforo_set_estado_operacao:
	push	lr

	mov r0, #PORT_OUT_LED_MASK_VERMELHO_L2

	mov r1, #PORT_OUT_LED_MASK_OPERATION_MODE_LED_L3
	orr r0, r0, r1

	ldr r1,addr_semaforo_estado_anterior_2
	ldr r2,[r1]

	mov r3, #0
	cmp r2, r3
	bne semaforo_set_estado_operacao_c1

	mov r3, #PORT_OUT_LED_MASK_AMARELO_L1
	orr r0, r0, r3

	mov r2, #1
	b semaforo_set_estado_operacao_end

semaforo_set_estado_operacao_c1:
	mov r2, #0

semaforo_set_estado_operacao_end:
	str r2,[r1]
	bl outport_write

	pop pc


; Rotina:   semaforo_set_estado_configuração
; Descricao: Funcao de controlo dos leds quando sistema esta em modo configuração
 
semaforo_set_estado_configuracao:
	push	lr

	mov r0, #PORT_OUT_LED_MASK_CONFIGURATION_MODE_LED_L3

	ldr r1,addr_semaforo_estado_anterior_2
	ldr r2,[r1]

	mov r3, #0
	cmp r2, r3
	bne semaforo_set_estado_configuracao_c1

	mov r3, #PORT_OUT_LED_MASK_AMARELO_L1
	orr r0, r0, r3
	mov r3,  #PORT_OUT_LED_MASK_VERDE_L2
	orr r0, r0,r3

	mov r2, #1
	b semaforo_set_estado_configuracao_end

semaforo_set_estado_configuracao_c1:
	mov r2, #0

semaforo_set_estado_configuracao_end:
	str r2,[r1]
	bl outport_write

	pop pc

addr_semaforo_estado_anterior_2:
     .word semaforo_estado_anterior

; Rotina:    semaforo_set_leds
; Descricao: Ve em que estado esta selecionado e chama funcao correspondente(máquina de estados)
; Entradas:  -
; Saidas:    -
; Efeitos: 
semaforo_set_leds:
	push	lr
	push 	r0
	push    r1

	ldr r1,addr_semaforo_led_estado_3
	ldr r0,[r1]

	mov r2, #0
	cmp r0, r2
	bne semaforo_set_leds_c1
	; ESTADO: operacao
	bl semaforo_set_estado_operacao

	b semaforo_set_leds_end
semaforo_set_leds_c1:
	mov r2, #1
	cmp r0, r2
	bne semaforo_set_leds_c2
	; ESTADO: PEDESTRIAN

	b semaforo_set_leds_end

semaforo_set_leds_c2:
	mov r2, #2
	cmp r0, r2
	bne semaforo_set_leds_end
	; ESTADO: CONFIGURACAO
	bl semaforo_set_estado_configuracao
	
	b semaforo_set_leds_end
semaforo_set_leds_end:
    pop r1
	pop r0
	pop pc

addr_semaforo_led_estado_3:
     .word semaforo_led_estado


; Rotina:    semaforo_main_function
; Descricao: Processo para ser executado no main. responsavel por piscar leds
;verificação da variável trigger , se trigger tiver a 1 ,faz-se os set dos leds
semaforo_main_function:
	push lr
	push r0
	push r1

	ldr r1,addr_trigger_semaforo_timer
	ldr r0,[r1]

	mov r1, #1
	cmp r0, r1
	bne semaforo_main_function_end
	bl semaforo_set_leds

	ldr r1,addr_trigger_semaforo_timer
	mov r0, #0
	str r0, [r1]

semaforo_main_function_end:
	pop r1
	pop r0
	pop pc 


; Rotina:    semaforo_isr_timer_control
; Descricao: Executada pela interrupcao, decremetna o timer do semaforo e executa a funcao dos leds
 ; verificação de 0.5 em 0.5s , quando 0.5 s passarem utilizamos a variavel trigger e metemos-la a 1
 ; trigger a 1 , METE O LED a 1 , a verificação é feita na main function
semaforo_isr_timer_control:
	push	lr

	ldr r2,addr_semaforo_timer_2
	ldr r0,[r2]
	sub r0, r0, #1
	
	bne semaforo_isr_timer_control_if1_end
	
	;bl semaforo_set_leds
	ldr r1,addr_trigger_semaforo_timer
	mov r0, #1
	str r0,[r1]

	mov r0,#BLINK_TIME

semaforo_isr_timer_control_if1_end:
    str r0,[r2]
	pop pc

addr_trigger_semaforo_timer:
     .word trigger_semaforo_timer

addr_semaforo_timer_2:
     .word semaforo_t

; Rotina:    crossing_time_isr_timer_control
; Descricao: Executada pela interrupcao, decremetna o timer do semaforo e executa a funcao dos leds
;quando se carrega no botão vamos esperar o crossing_time até terminar
;se crossing time for 0 transita-se de estado par nomeadamente('OPERAÇÃO')
crossing_time_isr_timer_control:
	push	lr

	ldr r1,addr_crossing_time_timer
	ldr r0,[r1]
	mov r2, #0
	cmp r0, r2
	bne crossing_time_isr_timer_control_non_zero
	
	; Transitar estado
	ldr r1, addr_semaforo_led_estado
	ldr r2, [r1]
	mov r0, #1
	cmp r2, r0
	bne crossing_time_isr_timer_control_end
	bl semaforo_timer_init
	bl semaforo_led_estado_init

crossing_time_isr_timer_control_non_zero:
	sub r0, r0, #1
	str r0,[r1]

crossing_time_isr_timer_control_end:
	pop pc

; Rotina:    semaforo_timer_init
; Descricao: Incicializa o timer do semaforo Conta de 5 em 5.

semaforo_timer_init:
        push lr

		ldr r1,addr_semaforo_timer
		mov r0, #BLINK_TIME
		str r0,[r1]  ; limpando eventuais pedidos de interrupcao pendentes (escrevendo)

		pop pc

; Rotina:    crossing_timer_init
; Descricao: Incicializa o timer do crossing time.

crossing_timer_init:
        push lr

		ldr r1,addr_crossing_time_timer
		mov r0, #0
		str r0,[r1]  ; limpando eventuais pedidos de interrupcao pendentes (escrevendo)

		ldr r1,crossing_time_index_addr
		mov r0, #CROSSING_TIME_DEFAULT_INDEX
		str r0,[r1]

		pop pc

; Rotina:    crossing_timer_start
; Descricao: Inicia contagem.

crossing_timer_start:
        push lr
		
		ldr r1,crossing_time_index_addr
		ldr r2, [r1]

		ldr r1, cfg_times_addr
		ldrb r0, [r1, r2]

		ldr r1,addr_crossing_time_timer
		str r0,[r1]  ; limpando eventuais pedidos de interrupcao pendentes (escrevendo)

		pop pc

cfg_times_addr:
	.word	cfg_times

crossing_time_index_addr:
	.word	crossing_time_index

; Rotina:    semaforo_led_estado_init
; Descricao: Incicializa as variaveis de control do estado dos leds

semaforo_led_estado_init:
        push lr

		ldr r1,addr_semaforo_led_estado
		mov r0, #0
		str r0,[r1]  ; limpando eventuais pedidos de interrupcao pendentes (escrevendo)

		ldr r1,addr_semaforo_estado_anterior
		mov r0, #0
		str r0,[r1]

		pop pc

; Rotina:    semaforo_led_config_init
; Descricao: Incicializa as variaveis de control no entado config

semaforo_led_config_init:
        push lr

		ldr r1,addr_semaforo_led_estado
		mov r0, #2
		str r0,[r1]  ; limpando eventuais pedidos de interrupcao pendentes (escrevendo)

		ldr r1,addr_semaforo_estado_anterior
		mov r0, #0
		str r0,[r1]

		pop pc

addr_crossing_time_timer:
     .word crossing_time_timer

addr_semaforo_timer:
     .word semaforo_t

addr_semaforo_led_estado:
     .word semaforo_led_estado

addr_semaforo_estado_anterior:
     .word semaforo_estado_anterior

; Rotina:    isr
; Descricao: Incrementa o valor da variável global sysclk.
; Entradas:  -
; Saidas:    -
; Efeitos:  Incrementa o valor da variável global sysclk em 1 a cada interrupção.
isr:
   push lr
   push r0
   push r1
   push r2
   push r3
   bl   ptc_clr_irq
   bl   semaforo_isr_timer_control
   bl	crossing_time_isr_timer_control
   ldr  r1,addr_sysclk
   ldr  r0,[r1]
   add  r0 ,r0,#1
   str  r0,[r1]
   pop  r3
   pop  r2
   pop  r1
   pop  r0
   pop  lr
   movs pc,lr

; Rotina:    sysclk_init
; Descricao: Inicia uma nova contagem no periferico pTC com o intervalo de
;            contagem recebido em R0, em ticks, limpando eventuais pedidos de
;            interrupcao pendentes e iniciando com o valor zero a variavel
;            global sysclk.
;            Interface exemplo: void sysclk_init( uint8_t interval );
; Entradas:  R0 - Valor do novo intervalo de contagem, em ticks.
; Saidas:    -
; Efeitos:   Inicia a contagem no periferico a partir do valor zero, limpando
;            eventuais pedidos de interrupcao pendentes e iniciando com o
;            valor zero a variavel global sysclk
sysclk_init:
        push lr
		bl ptc_init
        ldr r1,addr_sysclk
		mov r0,#0
		str r0,[r1]  ; limpando eventuais pedidos de interrupcao pendentes (escrevendo)
		pop pc

; Rotina:    sysclk_get_ticks
; Descricao: Devolve o valor corrente da variável global sysclk.
;            Interface exemplo: uint16_t sysclk_get_ticks ( );
; Entradas:  -
; Saidas:   R0 - O valor corrente da variável global sysclk
; Efeitos:   -
sysclk_get_ticks:
                ldr r0,addr_sysclk
				ldr r0,[r0]
				mov pc,lr

addr_sysclk:
     .word sysclk
; Gestor de periférico para o porto de entrada
;

; Rotina:    inport_read
; Descricao: Adquire e devolve o valor corrente do porto de entrada.
;            Interface exemplo: uint8_t inport_read( );
; Entradas:  -
; Saidas:    R0 - valor adquirido do porto de entrada
; Efeitos:   -
inport_read:
	ldr	r1, inport_addr
	ldrb	r0, [r1, #0]
	mov	pc, lr

inport_addr:
	.word	INPORT_ADDRESS

; Gestor de periférico para o porto de saída
;


; Rotina:    outport_init
; Descricao: Faz a iniciacao do porto de saida, nele estabelecendo o valor
;            recebido em R0.
;            Interface exemplo: void outport_init( uint8_t value );
; Entradas:  R0 - Valor a atribuir ao porto de saida.
; Saidas:    -
; Efeitos:   Altera o valor da variavel global outport_img.
outport_init:
	push	lr
	ldr	r1, outport_img_addr
	strb	r0, [r1]
	bl	outport_write
	pop	pc

outport_img_addr:
	.word	outport_img

; Rotina:    outport_write
; Descricao: Escreve no porto de saida o valor recebido em R0.
;            Interface exemplo: void outport_write( uint8_t value );
; Entradas:  R0 - valor a atribuir ao porto de saida.
; Saidas:    -
; Efeitos:   -
outport_write:
	ldr	r1, outport_addr
	strb	r0, [r1, #0]
	mov	pc, lr

outport_addr:
	.word	OUTPORT_ADDRESS

; Gestor de periférico para o Pico Timer/Counter (pTC)
;

; Rotina:    ptc_init
; Descricao: Faz a iniciacao do periférico pTC, habilitando o seu funcionamento
;            em modo continuo e com o intervalo de contagem recebido em R0, em
;            ticks.
;            Interface exemplo: void ptc_init( uint8_t interval );
; Entradas:  R0 - Valor do novo intervalo de contagem, em ticks.
; Saidas:    -
; Efeitos:   Inicia a contagem no periferico a partir do valor zero, limpando
;            o pedido de interrupcao eventualmente pendente.
ptc_init:
    push    lr
	ldr	r1, PTC_ADDR
	mov	r2, #PTC_CMD_STOP
	strb	r2, [r1, #PTC_TCR]
	strb	r0, [r1, #PTC_TMR]
    bl  ptc_clr_irq
	mov	r2, #PTC_CMD_START
	strb	r2, [r1, #PTC_TCR]
	pop pc

; Rotina:    ptc_start
; Descricao: Habilita a contagem no periferico pTC.
;            Interface exemplo: void ptc_start( );
; Entradas:  -
; Saidas:    -
; Efeitos:   -
ptc_start:
	ldr	r0, PTC_ADDR
	mov	r1, #PTC_CMD_START
	strb	r1, [r0, #PTC_TCR]
	mov	pc, lr

; Rotina:    ptc_stop
; Descricao: Para a contagem no periferico pTC.
;            Interface exemplo: void ptc_stop( );
; Entradas:  -
; Saidas:    -
; Efeitos:   O valor do registo TC do periferico e colocado a zero.
ptc_stop:
	ldr	r0, PTC_ADDR
	mov	r1, #PTC_CMD_STOP
	strb	r1, [r0, #PTC_TCR]
	mov	pc, lr

; Rotina:    ptc_get_value
; Descricao: Devolve o valor corrente da contagem do periferico pTC.
;            Interface exemplo: uint8_t ptc_get_value( );
; Entradas:  -
; Saidas:    R0 - O valor corrente do registo TC do periferico.
; Efeitos:   -
ptc_get_value:
	ldr	r1, PTC_ADDR
	ldrb	r0, [r1, #PTC_TC]
	mov	pc, lr

; Rotina:    ptc_clr_irq
; Descricao: Sinaliza o periferico pTC que foi atendido um pedido de
;            interrupção.
;            Interface exemplo: void ptc_clr_irq( );
; Entradas:  -
; Saidas:    -
; Efeitos:   -
ptc_clr_irq:
	ldr	r0, PTC_ADDR
	strb	r1, [r0, #PTC_TIR]
	mov	pc, lr

PTC_ADDR:
	.word	PTC_ADDRESS

; Seccao:    data
; Descricao: Guarda as variáveis globais
;
	.data
outport_img:
	.space	1

	.align
sysclk:
	.space	2

	.align
semaforo_t:
	.space	2
	
	.align
crossing_time_timer:
	.space  2

	.align
semaforo_led_estado:
	.space	2

	.align
semaforo_estado_anterior:
	.space	2

	.align
trigger_semaforo_timer:
	.space 2

	.align
btn_estado:
	.space 2

	.align
crossing_time_index:
	.space 2

;array com 5 valores entre 10 e 60 s
cfg_times:      ; 10s(valor pré-definido) 12s  15s 17s  20s
	.byte  100, 120, 150, 170, 200


; Seccao:    stack
; Descricao: Implementa a pilha com o tamanho definido pelo simbolo STACK_SIZE
;
	.stack
	.space	STACK_SIZE
stack_top:
