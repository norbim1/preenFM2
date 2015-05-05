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

#ifndef LFOENV_H_
#define LFOENV_H_

#include "Lfo.h"
#include "Env.h"

//### ADDED ###
extern float lfoEnv1OscValues[];

class LfoEnv: public Lfo {
public:
    LfoEnv();

	void init(struct EnvelopeParams * envParams, Matrix* matrix, SourceEnum source, DestinationEnum dest);

	void valueChanged(int encoder) {
        switch (encoder) {
        case 0:
            stateInc[ENV_STATE_ON_A] = 0.0008f / (envParams->attack + 0.0001);
			//### ADDED ###
			attackVal = envParams->attack;
            break;
        case 1:
            stateInc[ENV_STATE_ON_D] = 0.0008f / (envParams->decay + 0.0001);
			//### ADDED ###
			decayVal = envParams->decay;
            break;
        case 2:
            stateTarget[ENV_STATE_ON_D] =  envParams->sustain;
            stateTarget[ENV_STATE_ON_S] =  envParams->sustain;
			//### ADDED ###
			sustainLevel = envParams->sustain;
            break;
        case 3:
            stateInc[ENV_STATE_ON_R] = 0.0008f / (envParams->release + 0.0001);
			//### ADDED ###
			releaseVal = envParams->release;
            break;
        }	}

	//### ADDED ###
	/*
	 * Sets the values of the Env Oscillator array based on the current envelope settings
	 */
	void updateOscValues()
	{
		// 512 positions - 2.56 max in value
		// limit
		if(attackVal > 2.56) attackVal = 2.56;
		if(decayVal > 2.56) decayVal = 2.56;
		if(releaseVal > 2.56) releaseVal = 2.56;
		// sustainLevel
		if(sustainLevel > 1) sustainLevel = 1;
		// attack
		int16_t attackCount = attackVal * 200;
		// decay
		int16_t decayCount = decayVal * 200;
		// release
		int16_t releaseCount = releaseVal * 200;
		// flat fill
		int16_t flatCount = 0;

		// adjustments:
		//  adjust last step to array limit
		//  earlier states hold preference
		int16_t total = attackCount + decayCount + releaseCount;
		if (total < 512)
		{
			flatCount = 512 - total;
		}
		else if ((attackCount + decayCount) < 512)
		{
			releaseCount = 512 - (attackCount + decayCount);
		}
		else if (attackCount < 512)
		{
			decayCount  = 512 - attackCount;
		}
		// a _ d s r
		//  /f\___
		// /      \
		// Assign values
		int16_t sampleIndex = 0;

		// attack samples
		//  0 for silenceCount
		int16_t breakIndex = attackCount;
		float sampleIncrement = 1.0f / attackCount;
		float currentValue = 0;
		for (; sampleIndex < 512 && sampleIndex < breakIndex; sampleIndex++)
		{
			lfoEnv1OscValues[sampleIndex] = currentValue;
			lfoEnv1OscValues[1023 - sampleIndex] = -currentValue;
			currentValue += sampleIncrement;
		}

		// decay samples
		//  0 to 1 for attackCount
		breakIndex += decayCount;
		sampleIncrement = (1.0f - sustainLevel) / decayCount;
		currentValue = 1;
		for (; sampleIndex < 512 && sampleIndex < breakIndex; sampleIndex++)
		{
			lfoEnv1OscValues[sampleIndex] = currentValue;
			lfoEnv1OscValues[1023 - sampleIndex] = -currentValue;
			currentValue -= sampleIncrement;
		}
		// flat fill
		//  1 for flatCount
		breakIndex += flatCount;
		for (; sampleIndex < 512 && sampleIndex < breakIndex; sampleIndex++)
		{
			lfoEnv1OscValues[sampleIndex] = sustainLevel;
			lfoEnv1OscValues[1023 - sampleIndex] = -sustainLevel;
		}

		// release
		//  1 to 0 for decayCount
		breakIndex += releaseCount;
		sampleIncrement = sustainLevel / releaseCount;
		currentValue = sustainLevel;
		for (; sampleIndex < 512 && sampleIndex < breakIndex; sampleIndex++)
		{
			lfoEnv1OscValues[sampleIndex] = currentValue;
			lfoEnv1OscValues[1023 - sampleIndex] = -currentValue;
			currentValue -= sampleIncrement;
		}
	}
	//#############

    void newState() {

        if (env.envState == ENV_STATE_DEAD) {
            env.currentValue = 0;
        }
        env.previousStateValue = env.currentValue;
        env.nextStateValue = stateTarget[env.envState];
        env.currentPhase = 0;
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
        env.envState = ENV_STATE_ON_A;
        newState();
	}

	void noteOff() {
		env.envState = ENV_STATE_ON_R;
		newState();
	}

	void midiClock(int songPosition, bool computeStep) {
	}

private:
    // target values of ADSR
    float stateTarget[ENV_NUMBER_OF_STATES];
    // float
    float stateInc[ENV_NUMBER_OF_STATES];

    EnvelopeParams* envParams;
	EnvData env;

	//### ADDED ###
	float attackVal;
	float decayVal;
	float sustainLevel;
	float releaseVal;
	//#############
};

#endif /* LFOENV_H_ */
