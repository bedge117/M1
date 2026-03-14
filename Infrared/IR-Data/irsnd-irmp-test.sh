#!/bin/bash

# 01 Sony needs address greater 0x0100
# 06 Recs80 3 address bits
# 08 Denon only even commands
# 0c Recs80ext 4 address bits
# 0d Nubert 0 address bits
# 0e B&O 455kHz, 0 address bits
# 0f Grundig 0 address bits
# 11 Siemens letztes Bit invertiert zu vorletztem
# 12 FDC ziemlich speziell, Adresse, Kommando geht nicht?!
# 13 RCCAR 0 address bits, aber 1f -> 03
# 14 JVC 0 address bits, aber 1f -> 0f
# 16 Nikon 0 address bits, 2 data bits
# 17 Ruwido conflicts with denon, 56kHz
# 18 IR-60, 0 address bits, Wiederholung geht nicht
# 19 Kathrein can't send
# 1a Netbox can't send
# 1d Lego 0 address bits
# 1e Thomson 0 address bits, aber 1f -> f
# 1f Bose 0 address bits
# 20 A1TVBox 1f -> df
# 21 Ortek can't send
# 22 Telefunken 0 address bits
# 23 Roomba conflicts with RC6
# 24 RCMM32 can't send
# 25 RCMM24 can't send
# 26 RCMM12 can't send
# 27 Speaker 0 address bits
# 29 Samsung48 doppelt
# 2a Merlin 32bit
# 2b Pentax max 16kHz
# 2c Fan conflicts with Nubert
# 2d S100 conflicts with RC5
# 2e ACP24 conflicts with Denon
# 2f Technics can't send
# 30 Panasonic conflicts with Kaseikyo and Mitsu-Heavy 
# 31 Mitsu-Heavy geht nicht
for irdata in	"01 081f 003f 01" \
		"02 001f 003f 01" \
		"03 001f 003f 01" \
		"04 001f 003f 01" \
		"05 001f 003f 01" \
		"06 001f 003f 01" \
		"07 001f 003f 01" \
		"08 001f 003e 01" \
		"09 001f 003f 01" \
		"10 001f 003f 01" \
		"11 001f 003f 01" \
		"12 001f 003f 01" \
		"13 001f 003f 01" \
		"14 001f 003f 01" \
		"15 001f 003f 01" \
		"16 001f 003f 01" \
		"17 001f 003e 01" \
		"18 001f 003f 01" \
		"19 001f 003f 01" \
		"20 001f 003f 01" \
		"21 001f 003f 01" \
		"22 001f 003f 01" \
		"24 001f 003f 01" \
		"27 001f 003f 01" \
		"28 001f 003f 01" \
		"29 001f 003f 01" \
		"30 001f 003f 01" \
		"31 001f 003f 01" \
		"32 001f 003f 01" \
		"34 001f 003f 01" \
		"39 001f 003f 01" \
		"40 001f 003f 01" \
		"41 001f 003f 01" \
		"49 001f 003f 01";
do
	echo "${irdata}"
	tmpsrc/irsnd-20kHz ${irdata} | tmpsrc/irmp-20kHz
	#sleep 0.3
done
