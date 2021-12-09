;Sonden-Programm von L3P3 für CanSat 2015
;Version 0.0

;ATmega2560
	.include "m2560def.inc"
;Eigene Konstanten
	;Magische Zahl
		.set magic=183
	;Zeitskalierungen
		.set blocksPerSave=150+1
		.set blocksPerAlarmTick=5
		.set timeDef_a_def=4-1
		.set timeDef_b_def=130
		.set measuresPerBlock=64
		.set communDelay=6;Ab- statt aufgerundet!
	;Summ-Töne
		.set buzzDelA=8-1
		.set buzzDelB=4-1
	;Voreinstellungen
		.set preset_buzzEnabled=0
		.set preset_buzzToggleEnabled=0
		.set preset_measureEnabled=1
		.set preset_blinkEnabled=0
		.set preset_blinkIntEnabled=1
		.set preset_blinkIntDirection=0
		.set preset_connected=0
		.set uartNum=0;0: Kabel, 1: Funk
;UART-Auswahl
	.if uartNum==0
		.set ubRRnl=ubRR0l
		.set ubRRnh=ubRR0h
		.set ucsRnb=ucsR0b
		.set ucsRnc=ucsR0c
		.set ucsRna=ucsR0a
		.set udRn=udR0
		.set uart0deactivated=0
		.set uart1deactivated=1
	.elif uartNum==1
		.set ubRRnl=ubRR1l
		.set ubRRnh=ubRR1h
		.set ucsRnb=ucsR1b
		.set ucsRnc=ucsR1c
		.set ucsRna=ucsR1a
		.set udRn=udR1
		.set uart0deactivated=1
		.set uart1deactivated=0
	.endif
.eSeg;EEPROM
	.org 0x000
		eeprom_magic: .db magic
		eeprom_block_num: .dw 0
.dSeg;Arbeitsspeicher-Adressen
	.org 0x200
		measures_0: .byte 16
		measures_1: .byte 16
		measures_2: .byte 16
		measures_3: .byte 16
		measures_4: .byte 16
		eepExt_page: .byte 2
	.org 0x300
		commun_char_data: .byte 253
	.org 0x400
		eepExt_page_buffer: .byte 256
.cSeg;Code
;Register-Namen
	;P-Register
		.def ir_ram_cache=r0
		.def ir_sreg=r1
		.def ir_tmp=r2
		.def loop_block_counter=r3
		.def loop_measure_counter=r4
		.def commun_char_checksum=r5
		.def commun_char_lastReceived=r6
		.def block_nextSave=r7
		.def timeDef_b=r8
		.def commun_char_incoming_signal=r9
		.def alarm_buzz_tone=r10
		.def alarm_blink_state=r11
		.def flags_slow=r12
		.def eepExt_addr_a=r13
		.def eepExt_addr_b=r14
		.def eepExt_addr_c=r15
	;Normale Register
		.def ram_cache=r16
		.def tmp=r17
		.def portH_cache=r18
		.def portL_cache=r19
		.def flags_a=r20
		.def flags_b=r21
		.def commun_char_left=r22
		.def alarm_buzz_counter=r23
		.def block_num_l=r24
		.def block_num_h=r25
;Interrupt-Vektoren
	.org 0;Start
		rjmp start
	;Timer0
		.org OC0Aaddr-1
			rjmp OC0Aaddr_after
		.org OC0Aaddr
			rjmp ir_timer0
		OC0Aaddr_after:
	;UART 0
	.if uartNum==0
		.org URXC0addr-1
			rjmp URXC0addr_after
		.org URXC0addr;Ankommendes Byte
			rjmp ir_uart_receive
		URXC0addr_after:
	.endif
	;ADC
		.org ADCCaddr-1
			rjmp ADCCaddr_after
		.org ADCCaddr;ADC fertig (Zum Aufwecken)
			reti
		ADCCaddr_after:
	;UART 1
	.if uartNum==1
		.org URXC1addr-1
			rjmp URXC1addr_after
		.org URXC1addr;Ankommendes Byte
			rjmp ir_uart_receive
		URXC1addr_after:
	.endif
;Makros
	;IO-Funktionen
		.macro outI
			ldi ram_cache,@1
			out @0,ram_cache
		.endm
	;Gerät ausschalten
		.macro shutdown
			;Energiesparmodus: Alles aus (010)
				outI smcR,(0<<SM2)|(1<<SM1)|(0<<SM0)|(1<<SE)
			;Immerwieder ausschalten
				shutdownAgain:
				sleep
				rjmp shutdownAgain
		.endm
	;Register
		.macro setFlags
			ldi @0,(@1<<7)|(@2<<6)|(@3<<5)|(@4<<4)|(@5<<3)|(@6<<2)|(@7<<1)|(@8<<0)
		.endm
		.macro getP
			mov ram_cache,@0
		.endm
		.macro setP
			mov @0,ram_cache
		.endm
		.macro setPI
			ldi ram_cache,@1
			setP @0
		.endm
		;Flaggen
			.macro sbrP
				getP @0
				sbr ram_cache,@1
				setP @0
			.endm
			.macro cbrP
				getP @0
				cbr ram_cache,@1
				setP @0
			.endm
	;Arbeitsspeicher-Funktionen
		;Schreiben/Lesen
			.macro getRam
				lds ram_cache,@0
			.endm
			.macro setRam
				sts @0,ram_cache
			.endm
			.macro setRamI
				ldi ram_cache,@1
				setRam @0
			.endm
		;Flaggen
			.macro sbrRam
				getRam @0
				sbr ram_cache,@1
				setRam @0
			.endm
			.macro cbrRam
				getRam @0
				cbr ram_cache,@1
				setRam @0
			.endm
	;SPI
		;SPI öffnen
			.macro spiOpen
				cbr portH_cache,(1<<7)
				sts portH,portH_cache
			.endm
		;SPI schließen
			.macro spiClose
				sbr portH_cache,(1<<7)
				sts portH,portH_cache
			.endm
		;Warten, bis SPI fertig ist
			.macro spiWfr
				loop_spiWfr:
				getRam spsR
				sbrc ram_cache,SPIF
				rjmp loop_spiWfr
			.endm
		;Über SPI senden
			.macro spiSend
				out spdR,@0
				spiWfr
			.endm
			.macro spiSendI
				ldi ram_cache,@0
				spiSend ram_cache
			.endm
		;SPI empfangen
			.macro spiReceive
				spiSend alarm_buzz_tone;Einfach nur so
				in @0,spdR
			.endm
	;ADC-Messergebnis zu Zähler hinzufügen
		.macro adc_grabResult
			;Warten, bis ADC bereit
				adc_wait:
				sleep
				getRam adCsRA
				sbrc ram_cache,ADIF
				rjmp adc_wait
			;Ergebnis-Bytes dazuaddieren
				getRam @0
				lds tmp,adcL
				add ram_cache,tmp
				setRam @0
				getRam @0+1
				lds tmp,adcH
				adc ram_cache,tmp
				setRam @0+1
		.endm
	;LEDs setzen
		.macro setLEDs
			;PortH
				;Bits klären
					andi portH_cache,0b11101010
				;5g0->H4 (4)
					bst @0,5
					bld portH_cache,4
				;7r0->H2 (2)
					bst @0,7
					bld portH_cache,2
				;3w0->H0 (0)
					bst @0,3
					bld portH_cache,0
				;Setzen
					sts portH,portH_cache
			;PortL
				;Bits klären
					andi portL_cache,0b11101010
				;6r1->L4 (12)
					bst @0,6
					bld portL_cache,4
				;2w1->L2 (10)
					bst @0,2
					bld portL_cache,2
				;4g1->L0 (8)
					bst @0,4
					bld portL_cache,0
				;Setzen
					sts portL,portL_cache
		.endm
	;Interne LEDs setzen
		.macro setLEDInts
			out portA,@0
		.endm
	;Warten, bis externer EEPROM fertig ist
		.macro eepExtWfr
			;SPI öffnen
				spiOpen
			;Bytes senden
				spiSendI 0xd7
				eepExtWfr_wait:
					spiReceive ram_cache
					sbrs ram_cache,7
					rjmp eepExtWfr_wait
			;SPI schließen
				spiClose
		.endm
	;Interner EEPROM
		;Warten, bis bereit
			.macro eepIntWfr
				eepIntWrite_wait:
					sbic eecR,EEPE
					rjmp eepIntWrite_wait
			.endm
		;Byte schreiben
			.macro eepIntWrite
				;Warten, bis EEPROM bereit
					eepIntWfr
				;Adresse
					outI eeaRl,low(@0)
				;Daten
					out eedR,@1
				;Zeitkritisch: Kein Interrupt
					cli
				;Schreiben aktivieren
					sbi eecR,EEMPE
					sbi eecR,EEPE
				;Interrupts wieder aktivieren
					sei
			.endm
		;Byte lesen
			.macro eepIntRead
				;Adresse
					outI eeaRl,low(@1)
				;Lesen aktivieren
					sbi eecR,EERE
				;Daten
					in @0,eedR
			.endm
		;Aktuelle Blocknummer im EEPROM speichern
			.macro storage_state_save
				;Prüfzahl
					ldi tmp,magic
					eepIntWrite eeprom_magic,tmp
				;blockL
					eepIntWrite eeprom_block_num,block_num_l
				;blockH
					eepIntWrite eeprom_block_num+1,block_num_h
			.endm
		;EEPROM-Seite bis aktuellen Block speichern
			.macro eepExt_page_save
				;Zeiger
					clr tmp
					clr Yl
				;Befehl senden
					spiWfr
					spiOpen
					spiSendI 130
					spiSend eepExt_addr_a
					spiSend eepExt_addr_b
					spiSend tmp
				;Blöcke senden
					eepExt_page_save_blockLoop:
					;Block-Bytes
						;Byte 0
							ld ram_cache,y+
							spiSend ram_cache
						;Byte 1
							ld ram_cache,y+
							spiSend ram_cache
						;Byte 2
							ld ram_cache,y+
							spiSend ram_cache
						;Byte 3
							ld ram_cache,y+
							spiSend ram_cache
						;Byte 4
							ld ram_cache,y+
							spiSend ram_cache
						;Byte 5
							ld ram_cache,y+
							spiSend ram_cache
						;Byte 6
							ld ram_cache,y+
							spiSend ram_cache
						;Byte 7
							ld ram_cache,y+
							spiSend ram_cache
					;Prüfen, ob Zähler=Aktueller Block
						mov ram_cache,tmp
						sub ram_cache,eepExt_addr_c
						breq eepExt_page_save_blockLoop_after
					;Zähler weiterzählen
						ldi ram_cache,8
						add tmp,ram_cache
					;Schleife wiederholen
						rjmp eepExt_page_save_blockLoop
					eepExt_page_save_blockLoop_after:
				;Fertig
					spiClose
			.endm
			;Durchschnittswert auf 12bit kürzen
			;tmp=H, ram_cache=L
				.macro shortMeasure;Left?
					lds ram_cache,@0+0
					.if @1==1
						andi ram_cache,0b11110000
					.else
						lds tmp,@0+1
						;1
							clc
							ror tmp
							ror ram_cache
						;2
							clc
							ror tmp
							ror ram_cache
						;3
							clc
							ror tmp
							ror ram_cache
						;4
							clc
							ror tmp
							ror ram_cache
						sts @0+1,tmp
					.endif
					sts @0+0,ram_cache
				.endm
	;UART
		;Kommunikations-Status setzen
			.macro commun_char_state_set
				.if @0!=0b111
					cbr flags_b,0b111
				.endif
				.if @0!=0b000
					sbr flags_b,@0
				.endif
			.endm
		;Nachrichten versenden
			;Ungültige Nachricht empfangen
				.macro uart_send2
					;Nachricht in Zwischenspeicher schreiben
						;Magische Zahl
							setRamI commun_char_data+0,magic
						;Signal
							setRamI commun_char_data+1,2
						;Prüfsumme
							setRamI commun_char_data+2,181
					;Zeiger auf Anfang
						clr Xl
					;Nachrichten-Länge einstellen
						ldi commun_char_left,2+1
					;Eingang deaktivieren
						cbr flags_a,(1<<7)
					;Status einstellen
						commun_char_state_set 0b010
				.endm
	;Beim Ändern des Summ-Tones
		.macro alarm_buzz_tone_set
			mov alarm_buzz_tone,@0
			mov alarm_buzz_counter,@0
		.endm
		.macro alarm_buzz_tone_setIG
			ldi alarm_buzz_counter,@0
			mov alarm_buzz_tone,alarm_buzz_counter
		.endm
	;Interrupt
		;Beginn
			.macro interruptStart
				in ir_sReg,sReg
				mov ir_tmp,tmp
				mov ir_ram_cache,ram_cache
			.endm
		;Ende
			.macro interruptEnd
				mov ram_cache,ir_ram_cache
				mov tmp,ir_tmp
				out sReg,ir_sReg
				reti
			.endm
;Initialisierung
	start:
	;Ausgangswerte
		;Register
			;Boolen
				setFlags flags_a,preset_connected,preset_buzzToggleEnabled,0,preset_buzzEnabled,0,1,0,timeDef_a_def
				setFlags flags_b,preset_blinkEnabled,preset_blinkIntEnabled,preset_blinkIntDirection,1,preset_measureEnabled,1,0,0
			.if preset_buzzEnabled==1
				clr alarm_buzz_counter
			.endif
			;Die H-Adressbits ändern sich nie
				ldi Xh,high(commun_char_data)
				ldi Yh,high(eepExt_page_buffer)
		;P-Register
			setPI block_nextSave,blocksPerSave
			setPI timeDef_b,timeDef_b_def
			setPI alarm_buzz_tone,buzzDelA
			setPI alarm_blink_state,0
			setPI flags_slow,0
			;Port-Register
				clr portH_cache
				clr portL_cache
	;Stapelzeiger
		outI spL,low(RAMEND)
		outI spH,high(RAMEND)
	;Interner EEPROM
		;Die H-Adressbits ändern sich hier auch nie
			outI eeaRh,high(eeprom_magic)
	;Ports konfigurieren
		;Ausgänge
			;Interne LEDs
				outI ddrA,0xff
				.if preset_blinkIntEnabled==0
					ser tmp
					setLEDInts tmp
				.endif
			;SPI: MOSI, SCK
				outI ddrB,(1<<2)|(1<<0)
			;SS: Ausgang
				setRamI ddrH,(1<<7)|(1<<4)|(1<<2)|(1<<0)
			;Summer: Ausgang
				setRamI ddrL,(1<<6)|(1<<4)|(1<<2)|(1<<0)
	;ADC
		;Referenz und Bit-Ausrichtung
			.set adMuxD=(0<<REFS1)|(1<<REFS0)|(1<<ADLAR)
			setRamI adMux,adMuxD
		;Vorteiler=64
			.set adCsRAD=(0<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0)
			;Aktiviert
				.set adCsRAD2=adCsRAD|(1<<ADEN)|(1<<ADSC)
			setRamI adCsRA,adCsRAD
		.set adCsRBD=0
		;ldi adCsRB,adCsRBD
	;UART
		;Baudrate
			;1200: 416
			;9600: 51
			.set ubRRnv=51
			setRamI ubRRnl,low(ubRRnv)
			setRamI ubRRnh,high(ubRRnv)
		;Format
			.set ucsRnbD=(1<<RXCIE1)
			;Aktiviert
				.set ucsRnbD2=ucsRnbD|(1<<RXEN1)|(1<<TXEN1)
			setRamI ucsRnb,ucsRnbD
			setRamI ucsRnc,(1<<UCSZ11)|(1<<UCSZ10)
	;SPI
		outI spcR,(1<<SPE)|(1<<MSTR)|(0<<SPR1)|(0<<SPR0)
	;Energie sparen
		;Energiesparmodus einstellen
			outI smcR,(1<<SE)
		;Nicht benötigtes deaktivieren
			;Generell
				setRamI prR0,(1<<prTwi)|(1<<prTim2)|(0<<prTim0)|(1<<prTim1)|(0<<prSpi)|(uart0deactivated<<prUsart0)
				setRamI prR1,(1<<prTim5)|(1<<prTim4)|(1<<prTim3)|(1<<prUsart3)|(1<<prUsart2)|(uart1deactivated<<prUsart1)
			;Digitale Pins bei ADCs
				setRamI didR0,(1<<ADC0D)|(1<<ADC0D)
				setRamI didR2,(1<<ADC14D)|(1<<ADC9D)|(1<<ADC8D)
	;Uhr
		;CTC-Modus
			outI tccR0a,(1<<WGM01)
		;*256
			outI tccR0b,(1<<CS02)
		;*4
			outI ocR0a,3
		;Interrupt aktivieren
			setRamI timsk0,(1<<OCIE0A)
	;Überprüfen, ob EEPROM konfiguriert wurde
		eepExtWfr
		spiOpen
		spiSendI 0xd7
		spiReceive tmp
		spiClose
		sbrc tmp,0
		rjmp eepExt256check_after
		;Wenn zu konfigurieren ist
			;Verbindung deaktivieren
				commun_char_state_set 0b000
			;Setzen
				spiOpen
				spiSendI 0x3d
				spiSendI 0x2a
				spiSendI 0x80
				spiSendI 0xa6
				spiClose
			;Warten
				eepExtWfr
			;Alarm
				alarm_buzz_tone_setIG 232
				;Summer an
					.if preset_buzzEnabled==0
						sbr flags_a,(1<<4)
					.endif
				;Wechsel aus
					.if preset_buzzToggleEnabled==1
						cbr flags_a,(1<<6)
					.endif
				;Blinken an
					.if preset_blinkIntEnabled==0
						sbr flags_b,(1<<6)
					.endif
				;Messen aus
					.if preset_measureEnabled==1
						cbr flags_b,(1<<3)
					.endif
				rjmp storage_state_load_after
		eepExt256check_after:
	;EEPROM-Daten laden: Prüfzahl korrekt?
		eepIntRead tmp,eeprom_magic
		subi tmp,magic
		brne storage_state_load_failed
		;Ja: Blocknummer laden
			eepIntRead block_num_l,eeprom_block_num
			eepIntRead block_num_h,eeprom_block_num+1
			rjmp storage_state_load_after
		;Nein
			storage_state_load_failed:
			clr block_num_l
			clr block_num_h
		storage_state_load_after:
	;Interrupts  aktivieren
		sei
	;UART aktivieren
		setRamI ucsRnb,ucsRnbD2
;Hauptschleife
	loop_main:
	;Block-Schleife
		;Zähler einstellen
			setPI loop_block_counter,blocksPerAlarmTick
		loop_block:
		;Nachricht empfangen?
			mov tmp,flags_b
			andi tmp,0b111
			subi tmp,0b001
			breq message_tack
			rjmp message_tack_after
			message_tack:
			;Ja: Je nach Signal handeln
				cli
				;Sprungadresse einstellen
					ldi Zl,low(message_receive_signals)
					add Zl,commun_char_incoming_signal
					ldi Zh,high(message_receive_signals)
					ldi tmp,0
					adc Zh,tmp
				;Zu Punkt springen
					ijmp
			;Signale
				message_receive_signals:
				rjmp message_receive_signal_0
				rjmp message_receive_signal_1
				rjmp message_receive_signal_2
				rjmp message_receive_signal_3
				rjmp message_receive_signal_4
				rjmp message_receive_signal_5
				rjmp message_receive_signal_6
				rjmp message_receive_signal_7
				rjmp message_receive_signal_8
				rjmp message_receive_signal_9
				rjmp message_receive_signal_a
				rjmp message_receive_signal_b
				rjmp message_receive_signal_c
				rjmp message_receive_signal_d
				rjmp message_receive_signal_e
				rjmp message_receive_signal_f
			;Signal 1: Verbindung starten
				message_receive_signal_1:
				sei
				sbr flags_a,(1<<7)
				rjmp message_answer_1
			;Signal 2: Verbindung trennen
				message_receive_signal_2:
				sei
				cbr flags_a,(1<<7)
				rjmp message_answer_1
			;Signal 3: Aktuelle Messzeit senden
				message_receive_signal_3:
				sei
				;Magische Zahl
					setRamI commun_char_data+0,magic
				;Signal
					setRamI commun_char_data+1,3
					setPI commun_char_checksum,180
				;Parameter
					sts commun_char_data+2,block_num_h
					eor commun_char_checksum,block_num_h
					sts commun_char_data+3,block_num_l
					eor commun_char_checksum,block_num_l
					;Unterteilung berechnen
						clr ram_cache
						bst flags_a,0
						bld ram_cache,6
						bst flags_a,1
						bld ram_cache,7
					sts commun_char_data+4,ram_cache
					eor commun_char_checksum,ram_cache
				;Prüfsumme
					sts commun_char_data+5,commun_char_checksum
				;Zeiger auf Anfang
					clr Xl
				;Nachrichten-Länge einstellen
					ldi commun_char_left,2+3+1
				rjmp message_setStateSend
			;Signal 4: Blöcke senden
				message_receive_signal_4:
				sei
				;Prüfsumme zurücksetzen (mag+sign)
					setPI commun_char_checksum,0xB3
				;Nachrichten-Länge einstellen
					ldi commun_char_left,2+3+1
				;Anzahl berechnen
					;Max. Anzahl=Aktuelle Blocknummer-Start
						mov tmp,block_num_l
						mov eepExt_addr_a,block_num_h
						;Messen aktiviert?
							sbrs flags_b,3
							rjmp message_receive_signal_4_noBlockNumDec
							;Ja: Blocknummer-1
								subi block_num_l,1
								clr tmp
								sbc eepExt_addr_a,tmp
							message_receive_signal_4_noBlockNumDec:
						;L
							getRam commun_char_data+1
							sub tmp,ram_cache
						;H
							getRam commun_char_data+0
							sbc eepExt_addr_a,ram_cache
					;Wenn maximale Anzahl negativ oder null, dann keine senden
						brmi message_receive_signal_4_noBlocks
						breq message_receive_signal_4_noBlocks
						rjmp message_receive_signal_4_noBlocks_after
					;Keine Blöcke
						message_receive_signal_4_noBlocks:
						;Blocknummer
							;H
								sts commun_char_data+2,block_num_h
								eor commun_char_checksum,block_num_h
							;L
								sts commun_char_data+3,block_num_l
								eor commun_char_checksum,block_num_l
						;Anzahl der Blöcke
							setRamI commun_char_data+4,0
						;Prüfsumme
							sts commun_char_data+5,commun_char_checksum
						rjmp message_receive_signal_4_send
						message_receive_signal_4_noBlocks_after:
					;Maximale Anzahl auf 255 begrenzen
						;Max. Anzahl>255?
							tst eepExt_addr_a
							breq message_receive_signal_4_max255_after
							;Ja: Max. Anzahl=255
								ldi tmp,0xff
						message_receive_signal_4_max255_after:
					;Maximale Anzahl auf 31 begrenzen (Wegen 8bit-Zeiger)
						;Max. Anzahl>31
							mov ram_cache,tmp
							subi ram_cache,31;Max. Anzahl-31
							brmi message_receive_signal_4_max31_after
							;Ja: Max. Anzahl=31
								ldi tmp,31
							message_receive_signal_4_max31_after:
					;Maximale Anzahl<Geforderte Anzahl?
						getRam commun_char_data+2
						sub ram_cache,tmp;Geforderte-Maximal
						;Geforderte<Maximal?
							brpl message_receive_signal_4_maxReq_after
							;Ja: Maximal=Geforderte
								lds tmp,commun_char_data+2
						message_receive_signal_4_maxReq_after:
				;Antwort-Parameter in Nachricht schreiben
					;Blocknummer
						;H
							getRam commun_char_data+0
							setRam commun_char_data+2
							eor commun_char_checksum,ram_cache
						;L
							getRam commun_char_data+1
							setRam commun_char_data+3
							eor commun_char_checksum,ram_cache
					;Anzahl an Blöcken
						sts commun_char_data+4,tmp
						eor commun_char_checksum,tmp
				;Erste Adresse berechnen
					clr eepExt_addr_a
					lds eepExt_addr_b,commun_char_data+0
					lds eepExt_addr_c,commun_char_data+1
					;3mal nach links schieben
						clc
						;1
							rol eepExt_addr_c
							rol eepExt_addr_b
							rol eepExt_addr_a
						;2
							rol eepExt_addr_c
							rol eepExt_addr_b
							rol eepExt_addr_a
						;3
							rol eepExt_addr_c
							rol eepExt_addr_b
							rol eepExt_addr_a
				;Zeiger setzen
					ldi Xl,5
				;Adresse senden
					spiWfr
					spiOpen
					spiSendI 3
					spiSend eepExt_addr_a
					spiSend eepExt_addr_b
					spiSend eepExt_addr_c
				;Blöcke abfangen und in Nachricht schreiben
					message_receive_signal_4_receiveBlock:
					;Messen aktiviert?
						sbrs flags_b,3
						rjmp message_receive_signal_4_fromRam_after
						;Ja: Block in RAM?
							getRam eepExt_page+0
							sub ram_cache,eepExt_addr_b
							brne message_receive_signal_4_fromRam_after
							getRam eepExt_page+1
							sub ram_cache,eepExt_addr_a
							brne message_receive_signal_4_fromRam_after
							;Ja: Block aus RAM laden
								;Adresse einstellen
									mov Yl,eepExt_addr_c
								;Byte 0
									ld ram_cache,y+
									eor commun_char_checksum,ram_cache
									st x+,ram_cache
								;Byte 1
									ld ram_cache,y+
									eor commun_char_checksum,ram_cache
									st x+,ram_cache
								;Byte 2
									ld ram_cache,y+
									eor commun_char_checksum,ram_cache
									st x+,ram_cache
								;Byte 3
									ld ram_cache,y+
									eor commun_char_checksum,ram_cache
									st x+,ram_cache
								;Byte 4
									ld ram_cache,y+
									eor commun_char_checksum,ram_cache
									st x+,ram_cache
								;Byte 5
									ld ram_cache,y+
									eor commun_char_checksum,ram_cache
									st x+,ram_cache
								;Byte 6
									ld ram_cache,y+
									eor commun_char_checksum,ram_cache
									st x+,ram_cache
								;Byte 7
									ld ram_cache,y
									eor commun_char_checksum,ram_cache
									st x+,ram_cache
								rjmp message_receive_signal_4_fromRom_after
						message_receive_signal_4_fromRam_after:
					;Block aus EEPROM auslesen
						;Byte 0
							spiReceive ram_cache
							eor commun_char_checksum,ram_cache
							st x+,ram_cache
						;Byte 1
							spiReceive ram_cache
							eor commun_char_checksum,ram_cache
							st x+,ram_cache
						;Byte 2
							spiReceive ram_cache
							eor commun_char_checksum,ram_cache
							st x+,ram_cache
						;Byte 3
							spiReceive ram_cache
							eor commun_char_checksum,ram_cache
							st x+,ram_cache
						;Byte 4
							spiReceive ram_cache
							eor commun_char_checksum,ram_cache
							st x+,ram_cache
						;Byte 5
							spiReceive ram_cache
							eor commun_char_checksum,ram_cache
							st x+,ram_cache
						;Byte 6
							spiReceive ram_cache
							eor commun_char_checksum,ram_cache
							st x+,ram_cache
						;Byte 7
							spiReceive ram_cache
							eor commun_char_checksum,ram_cache
							st x+,ram_cache
						message_receive_signal_4_fromRom_after:
					;Nachrichtenlänge erhöhen
						ldi ram_cache,8
						add commun_char_left,ram_cache
					;Zähler zählen
						dec tmp
						breq message_receive_signal_4_receiveBlock_after
					;Adresse erhöhen
						add eepExt_addr_c,ram_cache
						clr ram_cache
						adc eepExt_addr_b,ram_cache
						adc eepExt_addr_a,ram_cache
					;Schleife wiederholen
						rjmp message_receive_signal_4_receiveBlock
					message_receive_signal_4_receiveBlock_after:
					spiClose
				;Prüfsumme
					st x+,commun_char_checksum
				;Nachricht versenden
					message_receive_signal_4_send:
					;Magische Zahl
						setRamI commun_char_data+0,magic
					;Signal
						setRamI commun_char_data+1,0x04
					;Zeiger auf Anfang
						clr Xl
				rjmp message_setStateSend
			;Signal 5: Messen aktivieren
				message_receive_signal_5:
				sei
				sbr flags_b,(1<<3)
				rjmp message_answer_1
			;Signal 6: Messen deaktivieren
				message_receive_signal_6:
				sei
				cbr flags_b,(1<<3)
				eepExt_page_save
				rjmp message_answer_1
			;Signal 7: Summer aktivieren und Wechsel an
				message_receive_signal_7:
				sei
				clr alarm_buzz_counter
				sbr flags_a,(1<<6)|(1<<4)
				rjmp message_answer_1
			;Signal 8: Summer deaktivieren
				message_receive_signal_8:
				sei
				cbr flags_a,(1<<4)
				;Summer auf 0 stellen
					cbr portL_cache,(1<<6)
					sts portL,portL_cache
					cbr flags_a,(1<<3)
				rjmp message_answer_1
			;Signal 9: Blinker aktivieren
				message_receive_signal_9:
				sei
				sbr flags_b,(1<<7)
				rjmp message_answer_1
			;Signal a: Blinker deaktivieren
				message_receive_signal_a:
				sei
				clr tmp
				rjmp message_receive_signal_e_afterRam
			;Signal b: Sonde ausschalten
				message_receive_signal_b:
				sei
				sbrP flags_slow,(1<<2)
				rjmp message_answer_1
			;Signal c: EEPROM leeren (Blocknummer auf 0 setzen)
				message_receive_signal_c:
				sei
				clr block_num_l
				clr block_num_h
				storage_state_save
				rjmp message_answer_1
			;Signal d: Ton setzen
				message_receive_signal_d:
				sei
				getRam commun_char_data
				andi ram_cache,0b00111111
				cli
				;Notenadresse einstellen
					ldi Zl,low(noteTable*2)
					add Zl,ram_cache
					ldi Zh,high(noteTable*2)
					ldi tmp,0
					adc Zh,tmp
				;Note laden
					lpm ram_cache,z
				sei
					alarm_buzz_tone_set ram_cache
				;Summer an
					sbr flags_a,(1<<4)
				;Wechsel aus
					cbr flags_a,(1<<6)
				rjmp message_answer_1
			;Signal e: LEDs setzen
				message_receive_signal_e:
				sei
				lds tmp,commun_char_data+0
				message_receive_signal_e_afterRam:
				;Blinken aus
					cbr flags_b,(1<<7)
				;LEDs einstellen
					setLEDs tmp
				;Internes Blinken aktiviert?
					bst tmp,1
					bld flags_b,6
				;Interne Blinkrichtung
					bst tmp,0
					bld flags_b,5
				;Internes Blinken deaktiviert?
					brts message_answer_1
					ser tmp
					setLEDInts tmp
				rjmp message_answer_1
			;Signal f: Block-Nummer setzen
				message_receive_signal_f:
				lds block_num_h,commun_char_data
				lds block_num_l,commun_char_data+1
			;Signal 0: Nichts tun
				message_receive_signal_0:
				sei
			;Befehl erhalten
				message_answer_1:
				;Nachricht in Zwischenspeicher schreiben
					;Magische Zahl
						setRamI commun_char_data+0,magic
					;Signal
						setRamI commun_char_data+1,1
					;Prüfsumme
						setRamI commun_char_data+2,182
				;Zeiger auf Anfang
					;ldi Xl,low(commun_char_data)
					clr Xl
				;Nachrichten-Länge einstellen
					ldi commun_char_left,2+1
			;Status auf "Senden" stellen
				message_setStateSend:
				commun_char_state_set 0b011
			message_tack_after:
		;Auf Block-Zeit warten (Abweichung: 1sek/10min)
			rjmp block_next_wait_first
			block_next_wait:
				sleep
			block_next_wait_first:
				;Darf nächster Block beginnen?
					sbrs flags_a,2
					;Nein
						rjmp block_next_wait
					;Ja
						cbr flags_a,(1<<2)
		;Alle 150 Blöcke die Blocknummer speichern
			dec block_nextSave
			brne block_nextSave_after
			;Wenn 0
				storage_state_save
				setPI block_nextSave,blocksPerSave
			block_nextSave_after:
		;Messen aktiviert?
			sbrs flags_b,3
			;Nein
				rjmp loop_block_measure_after
			;Ja
		;Messen möglich? (Blocknummer<255^2)
			ldi tmp,0xff
			sub tmp,block_num_h
			brne loop_block_measurePossible
			ldi tmp,0xff
			sub tmp,block_num_l
			brne loop_block_measurePossible
			;Nein
				rjmp loop_block_measure_after
			;Ja:
				loop_block_measurePossible:
		;Messen
			;Erster Block in Laufzeit?
				sbrs flags_b,4
				;Nein: Blocknummer erhöhen
					adiw block_num_l,1
			;EEPROM-Adresse berechnen
				clr eepExt_addr_a
				mov eepExt_addr_b,block_num_h
				mov eepExt_addr_c,block_num_l
				;3mal nach links schieben
					clc
					;1
						rol eepExt_addr_c
						rol eepExt_addr_b
						rol eepExt_addr_a
					;2
						rol eepExt_addr_c
						rol eepExt_addr_b
						rol eepExt_addr_a
					;3
						rol eepExt_addr_c
						rol eepExt_addr_b
						rol eepExt_addr_a
			;Erster Block auf Seite?
				tst eepExt_addr_c
				brne block_pageSwitch_after
				;Ja: Neue Seite öffnen
					;Seitennummer speichern
						sts eepExt_page+0,eepExt_addr_b
						sts eepExt_page+1,eepExt_addr_a
					rjmp block_pageNew_after
				;Nein
				block_pageSwitch_after:
			;Erster Block in Laufzeit? 
				sbrs flags_b,4
				;Nein
					rjmp block_pageNew_after
				;Ja: Block laden
					;Zeiger
						clr tmp
						clr Yl
					;Befehl senden
						spiWfr
						spiOpen
						spiSendI 3
						spiSend eepExt_addr_a
						spiSend eepExt_addr_b
						spiSend tmp
					;Blöcke empfangen
						eepExt_page_load_blockLoop:
						;Block-Bytes
							;Byte 0
								spiReceive ram_cache
								st y+,ram_cache
							;Byte 1
								spiReceive ram_cache
								st y+,ram_cache
							;Byte 2
								spiReceive ram_cache
								st y+,ram_cache
							;Byte 3
								spiReceive ram_cache
								st y+,ram_cache
							;Byte 4
								spiReceive ram_cache
								st y+,ram_cache
							;Byte 5
								spiReceive ram_cache
								st y+,ram_cache
							;Byte 6
								spiReceive ram_cache
								st y+,ram_cache
							;Byte 7
								spiReceive ram_cache
								st y+,ram_cache
						;Prüfen, ob Zähler=Aktueller Block
							mov ram_cache,tmp
							sub ram_cache,eepExt_addr_c
							breq eepExt_page_load_blockLoop_after
						;Zähler weiterzählen
							ldi ram_cache,8
							add tmp,ram_cache
						;Schleife wiederholen
							rjmp eepExt_page_load_blockLoop
						eepExt_page_load_blockLoop_after:
				block_pageNew_after:
			;Nicht mehr der erste Block
				cbr flags_b,(1<<4)
			;Mess-Schleife
				;Zähler einstellen
					setPI loop_measure_counter,measuresPerBlock
				loop_measure:
				;Alle Sensoren auslesen
					;Sensor 1
						;MUX: 0;000000
							setRamI adCsRB,adCsRBD|(0<<MUX5)
							setRamI adCsRA,adCsRAD2|(0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0)
						;Ergebnis dazuaddieren
							adc_grabResult measures_1
					;Sensor 2
						;MUX: 2;000010
							;setRamIG adCsRB,adCsRBD|(0<<MUX5)
							setRamI adCsRA,adCsRAD2|(0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(1<<MUX1)|(0<<MUX0)
						;Ergebnis dazuaddieren
							adc_grabResult measures_2
					;Sensor 0
						;MUX: 14;100110
							setRamI adCsRB,adCsRBD|(1<<MUX5)
							setRamI adCsRA,adCsRAD2|(0<<MUX4)|(0<<MUX3)|(1<<MUX2)|(1<<MUX1)|(0<<MUX0)
						;Ergebnis dazuaddieren
							adc_grabResult measures_0
					;Sensor 3
						;MUX: 8;100000
							;setRamIG adCsRB,adCsRBD|(1<<MUX5)
							setRamI adCsRA,adCsRAD2|(0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0)
						;Ergebnis dazuaddieren
							adc_grabResult measures_3
					;Sensor 4
						;MUX: 9;100001
							;setRamIG adCsRB,adCsRBD|(1<<MUX5)
							setRamI adCsRA,adCsRAD2|(0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(1<<MUX0)
						;Ergebnis dazuaddieren
							adc_grabResult measures_4
				;Nach den Einzelmessungen aden=0
					setRamI adCsRA,adCsRAD
				;Schleifensteuerung
					dec loop_measure_counter
					breq loop_measure_leave
					rjmp loop_measure
				loop_measure_leave:
			;Durchschnitte machen
				shortMeasure measures_0,1
				shortMeasure measures_1,0
				shortMeasure measures_2,1
				shortMeasure measures_3,0
				shortMeasure measures_4,1
			;Block in Zwischenspeicher speichern
				;Block-Anfangs-Adresse einstellen
					mov Yl,eepExt_addr_c
				;Messung 0
					getRam measures_0+1
					st y+,ram_cache
					lds tmp,measures_0+0
				;Messung 1
					getRam measures_1+1
					or ram_cache,tmp
					st y+,ram_cache
					getRam measures_1+0
					st y+,ram_cache
				;Zur Fehlerprüfung
					;alarm_buzz_tone_set ram_cache
				;Messung 2
					getRam measures_2+1
					st y+,ram_cache
					lds tmp,measures_2+0
				;Messung 3
					getRam measures_3+1
					or ram_cache,tmp
					st y+,ram_cache
					getRam measures_3+0
					st y+,ram_cache
				;Messung 4
					getRam measures_4+1
					st y+,ram_cache
					getRam measures_4+0
					st y+,ram_cache
			;Letzter Block auf Seite?
				ldi ram_cache,256-8
				sub ram_cache,eepExt_addr_c
				breq loop_block_measure_save
				rjmp loop_block_measure_after
				;Ja: Seite speichern
					loop_block_measure_save:
					eepExt_page_save
			loop_block_measure_after:
		;Schleifensteuerung
			dec loop_block_counter
			breq loop_block_leave
			rjmp loop_block
		loop_block_leave:
	;Alarm-Stand berechnen
		;Blinker aktiviert?
			sbrs flags_b,7
			;Nein
				rjmp loop_main_alarm_blink_after
			;Ja
				;Letzter Eintrag?
				ldi tmp,35
				sub tmp,alarm_blink_state
				breq loop_main_alarm_blink_isZero
				;Nein: Weiterzählen
					inc alarm_blink_state
					rjmp loop_main_alarm_blink_isZero_after
				;Ja: Zurücksetzen
					loop_main_alarm_blink_isZero:
					clr alarm_blink_state
				loop_main_alarm_blink_isZero_after:
				;LEDs einstellen
					cli
					;Adresse einstellen
						ldi Zl,low(ledTable*2)
						add Zl,alarm_blink_state
						ldi Zh,high(ledTable*2)
						clr tmp
						adc Zh,tmp
					;Daten laden
						lpm ram_cache,z
					sei
					;LEDs setzen
						setLEDs ram_cache
			loop_main_alarm_blink_after:
		;Interner Blinker aktiviert?
			sbrs flags_b,6
			;Nein
				rjmp loop_main_alarm_blinkint_after
			;Ja
				mov tmp,flags_slow
				;Welche Richtung?
					sbrc flags_b,5
					rjmp loop_main_alarm_blinkInt_backward
					;Vorwärts: Letzter Eintrag?
						andi tmp,0x11
						subi tmp,0x11
						breq loop_main_alarm_blinkInt_forward_isZero
						;Nein: Weiterzählen
							inc flags_slow
							rjmp loop_main_alarm_blinkInt_backward_after
						;Ja: Zurücksetzen
							loop_main_alarm_blinkInt_forward_isZero:
							cbrP flags_slow,0b11
							rjmp loop_main_alarm_blinkInt_backward_after
					;Rückwärts: Erster Eintrag?
						loop_main_alarm_blinkInt_backward:
						andi tmp,0x11
						breq loop_main_alarm_blinkInt_backward_isZero
						;Nein: Weiterzählen
							dec flags_slow
							rjmp loop_main_alarm_blinkInt_backward_after
						;Ja: Zurücksetzen
							loop_main_alarm_blinkInt_backward_isZero:
							sbrP flags_slow,0b11
					loop_main_alarm_blinkInt_backward_after:
					;Interne LEDs einstellen
						cli
						;Adresse einstellen
							ldi Zl,low(ledIntTable*2)
							mov tmp,flags_slow
							andi tmp,0b11
							add Zl,tmp
							ldi Zh,high(ledIntTable*2)
							ldi tmp,0
							adc Zh,tmp
						;Daten laden
							lpm ram_cache,z
						;LEDs setzen
							setLEDInts ram_cache
						sei
				loop_main_alarm_blinkint_after:
		;Summer-Wechsel aktiviert?
			sbrs flags_a,6
			;Nein
				rjmp loop_main_alarm_buzz_toggle_after
			;Ja: Welcher Status?
				sbrs flags_a,5
				rjmp loop_main_alarm_buzz_toggle_b_set
				;Status 1: ->0/Ton A
					cbr flags_a,(1<<5)
					alarm_buzz_tone_setIG buzzDelA
					rjmp loop_main_alarm_buzz_toggle_after
				;Status 0: ->1/Ton B
					loop_main_alarm_buzz_toggle_b_set:
					sbr flags_a,(1<<5)
					alarm_buzz_tone_setIG buzzDelB
			loop_main_alarm_buzz_toggle_after:
	;Schleifensteuerung
		rjmp loop_main
;Interrupts
	;Bei Uhr-Überlauf
		ir_timer0:
		interruptStart
		;UART bereit?
			getRam ucsRna
			sbrs ram_cache,UDRE1
			;Nein
				rjmp commun_char_send_after
			;Ja: Status=Senden?
				mov tmp,flags_b
				ror tmp
				andi tmp,0b11
				subi tmp,1
				brne commun_char_send_after
				;Ja
					ld ram_cache,x+
					;Byte senden
						setRam udRn
						dec commun_char_left
					;Nachricht komplett?
						brne commun_char_send_after
						;Ja: Status=Bereit
							commun_char_state_set 0b100
							sbrs flags_slow,2
							rjmp commun_char_send_after
							shutdown
			commun_char_send_after:
		;Summer aktiviert?
			sbrs flags_a,4
			rjmp alarmBuzzTick_after
			;Ja: Summerstellungs-Zähler=0?
				tst alarm_buzz_counter
				breq alarmBuzzTick_zero
				;Nein
					dec alarm_buzz_counter
					rjmp alarmBuzzTick_after
				;Ja
					alarmBuzzTick_zero:
					;Zähler zurücksetzen
						mov alarm_buzz_counter,alarm_buzz_tone
					;Summer-Stellung?
						sbrc flags_a,3
						rjmp alarm_buzz_turnDown
						;0: ->1
							sbr portL_cache,(1<<6)
							sbr flags_a,(1<<3)
							rjmp alarm_buzz_turn_save
						;1: ->0
							alarm_buzz_turnDown:
							cbr portL_cache,(1<<6)
							cbr flags_a,(1<<3)
						;Setzen
							alarm_buzz_turn_save:
							sts portL,portL_cache
			alarmBuzzTick_after:
		;B weiterzählen
			dec timeDef_b
			brne timeDef_b_inc_after
			;B=0?
				;B zurücksetzen
					setPI timeDef_b,timeDef_b_def
				;Kommunikations-Tick: Status=1__?
					sbrs flags_b,2
					rjmp communDelay_tick_after
					;Ja: Status=100?
						mov tmp,flags_b
						andi tmp,0b011
						breq communDelay_tick_after
						;Nein: Zähler weiterzählen
							dec commun_char_lastReceived
							brne communDelay_tick_after
							;Wenn Zähler=0
								uart_send2
					communDelay_tick_after:
				;A=0?
					mov tmp,flags_a
					andi tmp,0b11
					breq timeDef_a_zero
					;Nein: A+
						dec flags_a
						rjmp timeDef_b_inc_after
					;Ja: A=0 und nächsten Block erlauben
						timeDef_a_zero:
						sbr flags_a,(1<<2)|timeDef_a_def
			timeDef_b_inc_after:
		interruptEnd
	;Bei ankommendem UART-Byte
		ir_uart_receive:
		interruptStart
		;Summ-Ton ändern (Prüfung)
			;inc alarm_buzz_tone
		;Zähler zurücksetzen
			setPI commun_char_lastReceived,communDelay
		;Je nach Status reagieren
			;Sprungadresse einstellen
				ldi Zl,low(char_receive_states)
				mov tmp,flags_b
				andi tmp,0b111
				add Zl,tmp
				ldi Zh,high(char_receive_states)
				ldi tmp,0
				adc Zh,tmp
			;Angekommenes Byte
				lds tmp,udRn
			;Zu Punkt springen
				ijmp
		;Staten
			char_receive_states:
			;000: Deaktiviert
				rjmp char_receive_states_after
			;001: Nachricht verarbeiten
				rjmp char_receive_states_after
			;010: Ungestört senden
				rjmp char_receive_states_after
			;011: Senden
				rjmp char_receive_state_error
			;100: Empfangsbereit
				rjmp char_receive_state100
			;101: Erwarte Prüfsumme
				rjmp char_receive_state101
			;110: Erwarte Parameter
				rjmp char_receive_state110
			;111: Magische Zahl empfangen; Erwarte Signal
				rjmp char_receive_state111
		;Status 100/Erwarte magische Zahl
			char_receive_state100:
			subi tmp,magic
			breq char_receive_state100_isMagic
			rjmp char_receive_state_error
			char_receive_state100_isMagic:
			;Wenn magische Zahl
				;Prüfsumme setzen
					setPI commun_char_checksum,magic
				;Status
					commun_char_state_set 0b111
				rjmp char_receive_states_after
		;Status 101/Erwarte Prüfsumme
			char_receive_state101:
			eor commun_char_checksum,tmp
			breq char_receive_state101_checkCorrect
			rjmp char_receive_state_error
			;Wenn Prüfsumme korrekt
				char_receive_state101_checkCorrect:
				commun_char_state_set 0b001
				rjmp char_receive_states_after
		;Status 110/Erwarte Parameter-Byte
			char_receive_state110:
			;Byte speichern
				st x+,tmp
			;Byte zur Prüfsumme hinzufügen
				eor commun_char_checksum,tmp
			;Letztes Parameter-Byte?
				dec commun_char_left
				breq char_receive_state110_last
				rjmp char_receive_states_after
				;Ja
					char_receive_state110_last:
					commun_char_state_set 0b101
					rjmp char_receive_states_after
		;Status 111/Erwarte Signal
			char_receive_state111:
			;Verbindung unterbrochen?
				sbrc flags_a,7
				rjmp char_receive_state111_checkDisconnect_after
				;Ja: Signal="Verbindung herstellen"?
					mov commun_char_incoming_signal,tmp
					subi tmp,0x01
					brne char_receive_state_error
					;Nein: Fehler
					mov tmp,commun_char_incoming_signal
					rjmp char_receive_state111_storeSignal_after
				char_receive_state111_checkDisconnect_after:
			;Signal speichern
				mov commun_char_incoming_signal,tmp
				char_receive_state111_storeSignal_after:
			;Prüfsumme
				eor commun_char_checksum,commun_char_incoming_signal
			;Parameter-Länge und neue B-Signale bestimmen: Signal=0____?
				andi tmp,0xf0
				brne char_receive_state111_0____not
				;Ja: 0 0100?
					mov tmp,commun_char_incoming_signal
					subi tmp,0b0100
					breq char_receive_state111_f0100
					;Nein: 0 11__?
						mov tmp,commun_char_incoming_signal
						andi tmp,0b1100
						subi tmp,0b1100
						brne char_receive_noParams
						;Nein: Parameter-Länge=0
						;Ja: 0 1100?
							mov tmp,commun_char_incoming_signal
							subi tmp,0b1100
							brne char_receive_state_error
							;Nein: Fehler
							;Ja: Parameter-Länge=0
								rjmp char_receive_noParams
					;Ja
						char_receive_state111_f0100:
						;Parameter-Länge=3
							;Zeiger auf Anfang
								clr Xl
							;Nachrichten-Länge einstellen
								ldi commun_char_left,3
							commun_char_state_set 0b110
							rjmp char_receive_states_after
				;Nein: =b____?
					char_receive_state111_0____not:
					subi tmp,0xb0
					brne char_receive_state_error
					;Nein: Fehler
					;Ja: b00__?
						mov tmp,commun_char_incoming_signal
						andi tmp,0b1100
						brne char_receive_state_error
						;Nein: Fehler
						;Ja: b001_?
							sbrs commun_char_incoming_signal,1
							rjmp char_receive_state111_b000_
							;Ja: b0011?
								sbrc commun_char_incoming_signal,0
								;Ja: Fehler
									rjmp char_receive_state_error
								;Nein: Parameter-Länge=2
									;Zeiger auf Anfang
										clr Xl
									;Nachrichten-Länge einstellen
										ldi commun_char_left,2
									setPI commun_char_incoming_signal,0b1111
									commun_char_state_set 0b110
									rjmp char_receive_states_after
							;Nein, b000_: Parameter-Länge=1
								char_receive_state111_b000_:
								;Zeiger auf Anfang
									clr Xl
								;Nachrichten-Länge einstellen
									ldi commun_char_left,1
								;b0001?
									sbrc commun_char_incoming_signal,0
									rjmp char_receive_state111_b0001
									;Nein:
										setPI commun_char_incoming_signal,0b1101
										commun_char_state_set 0b110
										rjmp char_receive_states_after
									;Ja:
										char_receive_state111_b0001:
										setPI commun_char_incoming_signal,0b1110
										commun_char_state_set 0b110
										rjmp char_receive_states_after
			;Wenn ungültiges Byte angekommen ist
				char_receive_state_error:
				uart_send2
				rjmp char_receive_states_after
			;Wenn keine Parameter folgen
				char_receive_noParams:
				commun_char_state_set 0b101
		char_receive_states_after:
		interruptEnd
;Datenteil
	noteTable:;Länge 64
		.db 255,255,246,232,219,206,195,184,173,163,154,145,137,129,122,115,109,102,97,91,86,81,76,72,68,64,60,57,54,50,48,45,42,40,37,35,33,31,29,28,26,24,23,22,20,19,18,17,16,15,14,13,12,11,11,10,9,9,8,8,7,7,6,6
	ledTable:;Länge: 36
		;Rot
			.db 0,(1<<7)
			.db 0,(1<<6)
			.db 0,(1<<7)
			.db 0,(1<<6)
			.db 0,(1<<7)
			.db 0,(1<<6)
		;Grün
			.db 0,(1<<5)
			.db 0,(1<<4)
			.db 0,(1<<5)
			.db 0,(1<<4)
			.db 0,(1<<5)
			.db 0,(1<<4)
		;Weiß
			.db 0,(1<<3)
			.db 0,(1<<2)
			.db 0,(1<<3)
			.db 0,(1<<2)
			.db 0,(1<<3)
			.db 0,(1<<2)
	ledIntTable:;Länge: 4
			.db 0b01110111,0b10111011
			.db 0b11011101,0b11101110
;EOF