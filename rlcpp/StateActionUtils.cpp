# include "StateActionUtils.h"

void copyAction( Action * aFROM, Action * aTO ) {

    if ( aFROM->discrete && aTO->discrete ) {

        aTO->discreteAction = aFROM->discreteAction ;

    }

    if ( aFROM->continuous && aTO->continuous && ( aFROM->actionDimension == aTO->actionDimension ) ) {

        for ( int a = 0 ; a < aFROM->actionDimension ; a++ ) {

            aTO->continuousAction[a] = aFROM->continuousAction[a] ;

        }

    }

}

void copyState( State * sFROM, State * sTO ) {

    if ( sFROM->discrete && sTO->discrete ) {

        sTO->discreteState = sFROM->discreteState ;

    }

    if ( sFROM->continuous && sTO->continuous && ( sFROM->stateDimension == sTO->stateDimension ) ) {

        for ( int s = 0 ; s < sFROM->stateDimension ; s++ ) {

            sTO->continuousState[s] = sFROM->continuousState[s] ;

        }

    }

	sTO->time_step = sFROM->time_step;
}