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

/*
 * Eveloppe with 3 states
 * S = silence
 * A = attack
 * R = release
 * After release go to S or finished
 */

#ifndef LFOENV2_H_
#define LFOENV2_H_

#include "Lfo.h"
#include "Env.h"

//### ADDED ###
extern float lfoEnv2OscValues[];

enum Env2State {
    ENV2_STATE_ON_S = 0,
    ENV2_STATE_ON_A,
    ENV2_STATE_ON_D,
    ENV2_STATE_DEAD,
    ENV2_NUMBER_OF_STATES
};

class LfoEnv2: public Lfo {
public:
    LfoEnv2();

	void init(struct Envelope2Params * envParams, Matrix* matrix, SourceEnum source, DestinationEnum dest);

	void valueChanged(int encoder) {
        switch (encoder) {
        case 0:
            stateInc[ENV2_STATE_ON_S] = 0.0008f / (envParams->silence + 0.0001);
			//### ADDED ###
			silenceVal = envParams->silence;
            break;
        case 1:
            stateInc[ENV2_STATE_ON_A] = 0.0008f / (envParams->attack + 0.0001);
			//### ADDED ###
			attackVal = envParams->attack;
            break;
        case 2:
            stateInc[ENV2_STATE_ON_D] = 0.0008f / (envParams->decay + 0.0001);
			//### ADDED ###
			decayVal = envParams->decay;
            break;
        }
	}

	//### ADDED ###

	//void updateOscValuesOld()
	//{
	//	// 512 positions - 2.56 max in value
	//	// limit
	//	if(silenceVal > 2.56) silenceVal = 2.56;
	//	if(attackVal > 2.56) attackVal = 2.56;
	//	if(decayVal > 2.56) decayVal = 2.56;
	//	// delay / silence
	//	int16_t silenceCount = silenceVal * 200;
	//	// attack
	//	int16_t attackCount = attackVal * 200;
	//	// decay
	//	int16_t decayCount = decayVal * 200;
	//	// flat fill
	//	int16_t flatCount = 0;
	//	// adjustments:
	//	//  adjust last step to array limit
	//	//  earlier states hold preference
	//	int16_t total = silenceCount + attackCount + decayCount;
	//	if (total < 512)
	//	{
	//		flatCount = 512 - total;
	//	}
	//	else if ((silenceCount + attackCount) < 512)
	//	{
	//		decayCount = 512 - (silenceCount + attackCount);
	//	}
	//	else if (silenceCount < 512)
	//	{
	//		attackCount  = 512 - silenceCount;
	//	}
	//	//     _
	//	// s a/f\d
	//	// __/   \
	//	// Assign values
	//	int16_t sampleIndex = 0;
	//	int16_t breakIndex = silenceCount;
	//	// silence samples
	//	//  0 for silenceCount
	//	for (; sampleIndex < 512 && sampleIndex < breakIndex; sampleIndex++)
	//	{
	//		lfoEnv2OscValues[sampleIndex] = 0.0f;
	//		lfoEnv2OscValues[1023 - sampleIndex] = 0.0f;
	//	}
	//	// attack samples
	//	//  0 to 1 for attackCount
	//	breakIndex += attackCount;
	//	float sampleIncrement = 1.0f / attackCount;
	//	float currentValue = 0;
	//	for (; sampleIndex < 512 && sampleIndex < breakIndex; sampleIndex++)
	//	{
	//		lfoEnv2OscValues[sampleIndex] = currentValue;
	//		lfoEnv2OscValues[1023 - sampleIndex] = -currentValue;
	//		currentValue += sampleIncrement;
	//	}
	//	// flat fill
	//	//  1 for flatCount
	//	breakIndex += flatCount;
	//	for (; sampleIndex < 512 && sampleIndex < breakIndex; sampleIndex++)
	//	{
	//		lfoEnv2OscValues[sampleIndex] = 1.0f;
	//		lfoEnv2OscValues[1023 - sampleIndex] = -1.0f;
	//	}
	//	// decay
	//	//  1 to 0 for decayCount
	//	breakIndex += decayCount;
	//	sampleIncrement = 1.0f / decayCount;
	//	currentValue = 1;
	//	for (; sampleIndex < 512 && sampleIndex < breakIndex; sampleIndex++)
	//	{
	//		lfoEnv2OscValues[sampleIndex] = currentValue;
	//		lfoEnv2OscValues[1023 - sampleIndex] = -currentValue;
	//		currentValue -= sampleIncrement;
	//	}
	//}

	/*
	 * Sets the values of the Env Oscillator array based on the current envelope settings
	 */
	void updateOscValues()
	{
		// 512 positions - 2.56 max in value
		// limit
		if(silenceVal > 2.56) silenceVal = 2.56;
		if(attackVal > 2.56) attackVal = 2.56;
		if(decayVal > 2.56) decayVal = 2.56;
		// delay / silence
		int16_t silenceCount = silenceVal * 400;
		// attack
		int16_t attackCount = attackVal * 400;
		// decay
		int16_t decayCount = decayVal * 400;
		// flat fill
		int16_t flatCount = 0;

		// adjustments:
		//  adjust last step to array limit
		//  earlier states hold preference
		int16_t total = silenceCount + attackCount + decayCount;
		if (total < 1024)
		{
			flatCount = 1024 - total;
		}
		else if ((silenceCount + attackCount) < 1024)
		{
			decayCount = 1024 - (silenceCount + attackCount);
		}
		else if (silenceCount < 1024)
		{
			attackCount  = 1024 - silenceCount;
		}
		//     _
		// s a/f\d
		// __/   \
		// Assign values
		int16_t sampleIndex = 0;
		int16_t breakIndex = silenceCount;

		// silence samples
		//  0 for silenceCount
		for (;sampleIndex < breakIndex && sampleIndex < 1024; sampleIndex++)
		{
			lfoEnv2OscValues[sampleIndex] = -1.0f;
		}

		// attack samples
		//  0 to 1 for attackCount
		breakIndex += attackCount;
		float sampleIncrement = 2.0f / attackCount;
		float currentValue = -1.0f;
		for (;sampleIndex < breakIndex && sampleIndex < 1024; sampleIndex++)
		{
			lfoEnv2OscValues[sampleIndex] = currentValue;
			currentValue += sampleIncrement;
		}
		// flat fill
		//  1 for flatCount
		breakIndex += flatCount;
		for (;sampleIndex < breakIndex && sampleIndex < 1024; sampleIndex++)
		{
			lfoEnv2OscValues[sampleIndex] = 1.0f;
		}

		// decay
		//  1 to 0 for decayCount
		breakIndex += decayCount;
		sampleIncrement = 2.0f / decayCount;
		currentValue = 1;
		for (;sampleIndex < breakIndex && sampleIndex < 1024; sampleIndex++)
		{
			lfoEnv2OscValues[sampleIndex] = currentValue;
			currentValue -= sampleIncrement;
		}
	}
	//#############

    void newState() {
        if (env.envState == ENV2_STATE_DEAD) {
            env.currentValue = 0;
            if (envParams->loop > 1) {
                env.envState = ENV2_STATE_ON_A;
            } else if (envParams->loop > 0) {
                env.envState = ENV2_STATE_ON_S;
            }
        }
        env.previousStateValue = env.currentValue;
        env.nextStateValue = stateTarget[env.envState];
        env.currentPhase = 0;
        if (env.envState == ENV2_STATE_ON_S) {
            if (this->matrix->getDestination(this->destination) != 0) {
                float silencePlusMatrix = envParams->silence + this->matrix->getDestination(this->destination);
                if (silencePlusMatrix <= 0) {
                    stateInc[ENV2_STATE_ON_S] = 1.0;
                } else if (silencePlusMatrix > 8) {
                    stateInc[ENV2_STATE_ON_S] = 0.0001f ;
                } else {
                    stateInc[ENV2_STATE_ON_S] = 0.0008f / silencePlusMatrix;
                }
            }
        }
    }



    void nextValueInMatrix() {
        env.currentPhase += stateInc[env.envState];

        if (env.currentPhase  >= 1.0f) {
            env.currentValue = env.nextStateValue;
            env.envState++;
            newState();
        } else if (stateInc[env.envState] > 0) {
            env.currentValue = env.previousStateValue * (1- env.currentPhase) + env.nextStateValue * env.currentPhase;
        }

		matrix->setSource(source, env.currentValue);
	}

	void noteOn() {
        // index is decremented in the first call...
        env.currentValue = 0;
        env.envState = ENV2_STATE_ON_S;
        newState();
	}

	void noteOff() {
	}

	void midiClock(int songPosition, bool computeStep) {
	}

private:
    // target values of ADSR
    float stateTarget[ENV2_NUMBER_OF_STATES];
    // float
    float stateInc[ENV2_NUMBER_OF_STATES];

    Envelope2Params* envParams;
	EnvData env;

	//### ADDED ###
	float silenceVal;
	float attackVal;
	float decayVal;
	//#############
};

#endif /* LfoEnv2_H_ */
