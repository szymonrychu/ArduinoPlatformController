#ifndef PeriodicFuncHandler_H
#define PeriodicFuncHandler_H

#include "pins.h"

#ifndef MAX_FUNC_HANDLER
#define MAX_FUNC_HANDLER 30
#endif

class PeriodicFuncHandler{
private:
    int loopCounter = 0;
    long overralCounter = 0;
    int currentFuncHandlerN = 0;
    void (*funcHandler[MAX_FUNC_HANDLER])();
    int funcHandlerSecDelta[MAX_FUNC_HANDLER];
    int funcHandlerInitialDelay[MAX_FUNC_HANDLER];
    int funcHandlerSecDeltaCounter[MAX_FUNC_HANDLER];
public:
    PeriodicFuncHandler(){
        currentFuncHandlerN = 0;
        overralCounter = 0;
    }

    void appendHandler(int initialDelayS, int secPeriodS, void (handler())){
        funcHandler[currentFuncHandlerN] = handler;
        
        funcHandlerSecDelta[currentFuncHandlerN] = secPeriodS;
        funcHandlerInitialDelay[currentFuncHandlerN] = initialDelayS;
        funcHandlerSecDeltaCounter[currentFuncHandlerN] = 0;
        currentFuncHandlerN++;
    }

    void tick(){
        loopCounter = (loopCounter + 1) % TICKS2SEC;
        if(loopCounter == 0){
            for(int c=0; c<currentFuncHandlerN; c++){
                if(funcHandlerInitialDelay[c] > 0){
                    funcHandlerInitialDelay[c] = funcHandlerInitialDelay[c] - 1;
                }else{
                    if(funcHandlerSecDeltaCounter[c] == 0){
                        // Serial.print("debug: PeriodicFuncHandler (");
                        // Serial.print(overralCounter);
                        // Serial.print("s): running handler ");
                        // Serial.print(c+1);
                        // Serial.print("/");
                        // Serial.print(currentFuncHandlerN);
                        // Serial.print(" ");
                        // Serial.print(funcHandlerInitialDelay[c]);
                        // Serial.print("/");
                        // Serial.print(funcHandlerSecDelta[c]);
                        // Serial.print("/");
                        // Serial.print((int)funcHandler[c]);
                        // Serial.println(";");
                        (funcHandler[c]());
                    }
                    funcHandlerSecDeltaCounter[c] = (funcHandlerSecDeltaCounter[c]+1) % funcHandlerSecDelta[c];
                }
            }
            overralCounter++;
        }
    }
};

#endif