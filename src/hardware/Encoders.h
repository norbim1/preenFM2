/*
 * Copyright 2013 Xavier Hosxe
 *
 * Author: Xavier Hosxe (xavier . hosxe (at) gmail . com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ENCODERS_H_
#define ENCODERS_H_

#include "RingBuffer.h"
#include "EncodersListener.h"
#include "stm32f4xx.h"

//#define HC165_CLOCK GPIO_Pin_8
//#define HC165_DATA  GPIO_Pin_7
//#define HC165_LOAD  GPIO_Pin_15

#define BUTTON0_PIN  GPIO_Pin_10
#define BUTTON0_PORT GPIOE
#define BUTTON1_PIN  GPIO_Pin_13
#define BUTTON1_PORT GPIOC
#define BUTTON2_PIN  GPIO_Pin_15
#define BUTTON2_PORT GPIOC
#define BUTTON3_PIN  GPIO_Pin_1
#define BUTTON3_PORT GPIOA
#define BUTTON4_PIN  GPIO_Pin_3
#define BUTTON4_PORT GPIOA
#define BUTTON5_PIN  GPIO_Pin_5
#define BUTTON5_PORT GPIOA
#define BUTTON6_PIN  GPIO_Pin_5
#define BUTTON6_PORT GPIOC
#define BUTTON7_PIN  GPIO_Pin_1
#define BUTTON7_PORT GPIOB

//#define BUTTON_PORT GPIOE

#define ENC1_0_PIN  GPIO_Pin_2
#define ENC1_0_PORT GPIOC
#define ENC1_2_PIN  GPIO_Pin_1
#define ENC1_2_PORT GPIOC
#define ENC2_0_PIN  GPIO_Pin_4
#define ENC2_0_PORT GPIOC
#define ENC2_2_PIN  GPIO_Pin_2
#define ENC2_2_PORT GPIOA
#define ENC3_0_PIN  GPIO_Pin_4
#define ENC3_0_PORT GPIOE
#define ENC3_2_PIN  GPIO_Pin_2
#define ENC3_2_PORT GPIOE
#define ENC4_0_PIN  GPIO_Pin_14
#define ENC4_0_PORT GPIOC
#define ENC4_2_PIN  GPIO_Pin_6
#define ENC4_2_PORT GPIOE

//#define ENC_PORT GPIOD

#define NUMBER_OF_ENCODERS 4
#define NUMBER_OF_BUTTONS 8

enum LastEncoderMove {
	LAST_MOVE_NONE = 0,
	LAST_MOVE_INC,
	LAST_MOVE_DEC
};


struct EncoderStatus {
	char value;
	bool b1;
};

class Encoders {
public:
	Encoders();
	~Encoders();
	void checkStatus(int encoderType);
	void checkSimpleStatus();
	int getRegisterBits();

	void insertListener(EncodersListener *listener) {
		if (firstListener!=0) {
			listener->nextListener = firstListener;
		}
		firstListener = listener;
	}

	void encoderTurned(int encoder, int ticks) {
		if (firstButtonDown == -1) {
			for (EncodersListener* listener = firstListener; listener !=0; listener = listener->nextListener) {
				listener->encoderTurned(encoder, ticks);
			}
		} else {
			for (EncodersListener* listener = firstListener; listener !=0; listener = listener->nextListener) {
				listener->encoderTurnedWhileButtonPressed(encoder, ticks, firstButtonDown);
			}
			buttonUsedFromSomethingElse[firstButtonDown] = true;
		}
	}

	void encoderTurnedWileButtonDown(int encoder, int ticks) {
		for (EncodersListener* listener = firstListener; listener !=0; listener = listener->nextListener) {
			listener->encoderTurned(encoder, ticks);
		}
	}


	void buttonPressed(int button) {
		for (EncodersListener* listener = firstListener; listener !=0; listener = listener->nextListener) {
			listener->buttonPressed(button);
		}
	}

	void twoButtonsPressed(int button1, int button2) {
		for (EncodersListener* listener = firstListener; listener !=0; listener = listener->nextListener) {
			listener->twoButtonsPressed(button1, button2);
		}
	}

private:
	int action[2][16];
	unsigned short encoderBit1[NUMBER_OF_ENCODERS];
	unsigned short encoderBit2[NUMBER_OF_ENCODERS];
	GPIO_TypeDef* encoderPort1[NUMBER_OF_ENCODERS];
	GPIO_TypeDef* encoderPort2[NUMBER_OF_ENCODERS];
	int encoderState[NUMBER_OF_ENCODERS];
	int timerAction[NUMBER_OF_ENCODERS];

	LastEncoderMove lastMove[NUMBER_OF_ENCODERS];
    int tickSpeed[NUMBER_OF_ENCODERS];

    unsigned short buttonBit[NUMBER_OF_BUTTONS];
    GPIO_TypeDef* buttonPort[NUMBER_OF_BUTTONS];
    int buttonTimer[NUMBER_OF_BUTTONS];
	bool buttonUsedFromSomethingElse[NUMBER_OF_BUTTONS];
	bool buttonPreviousState[NUMBER_OF_BUTTONS];
	int firstButtonDown;

	int encoderTimer;

	EncodersListener* firstListener;
};

#endif /* ENCODERS_H_ */
