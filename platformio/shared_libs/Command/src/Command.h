#ifndef COMMAND_H
#define COMMAND_H
#include <string.h>

#ifndef MEM_LEN
#define MEM_LEN 256
#endif
#define MAXSERIALCOMMANDS	10
#define MAXDELIMETER 2

class Command{
private:
	char *receiveCache;
	char *errorCache;
	int numCommand;
	char *last;
	char *token;

	void (*defaultHandler)(char*);
    bool defaultHandlerExists = false;

	typedef struct _callback {
		char command[MEM_LEN];
		void (*function)();
	} CommandCallback;
	CommandCallback CommandList[MAXSERIALCOMMANDS];

protected:
	int  bufPos;
	char buffer[MEM_LEN];
	bool termFound = false;
	char *delim;
	char *term;
    bool integrityHandler(char* data){
        return true;
    }
public:

	Command(char *delimeter, char *terminator){
		delim = delimeter;
		term = terminator;
		memset(buffer, 0, MEM_LEN);
	}


	void inputChar(char inChar){
		if(13 == (int)inChar)return; //discard carriage returns
		if(inChar == term[0]) {
			termFound = true;
			receiveCache = (char*)calloc(strlen(&(buffer[0]))+1, sizeof(char));
			errorCache = (char*)calloc(strlen(&(buffer[0]))+1, sizeof(char));
			strcpy(receiveCache, &(buffer[0]));
			strcpy(errorCache, &(buffer[0]));
			bufPos = 0;
			memset(buffer, 0, MEM_LEN);
		}else{
			buffer[bufPos++]=inChar;
			buffer[bufPos]='\0';
			if (bufPos > MEM_LEN-1) bufPos=0;
		}
	}

    void inputString(char* data){
        for(int c=0; c<strlen(data); c++){
            inputChar(data[c]);
        }
    }

	bool lastPinState = false;
	int counter = 0;
	void toggleLED(){
		if(counter == 0){
			lastPinState = ! lastPinState;
			if(lastPinState){
				digitalWrite(LED_BUILTIN, HIGH);
			}else{
				digitalWrite(LED_BUILTIN, LOW);
			}
		}
		counter = (counter + 1)%10;
	}

	void parse(){
		toggleLED();
		char *tmp;
		if(termFound){
			termFound = false;
            if(!integrityHandler(receiveCache) ){
                if(defaultHandlerExists){
                    (*defaultHandler)(errorCache);
                }
                return;
            }
			token = strtok_r(receiveCache, delim, &last);
			if (token == NULL){
                if(defaultHandlerExists){
                    (*defaultHandler)(errorCache);
                }
				return;
			}
			bool matched = false;
			for (int i=0; i<numCommand; i++) {
				if (strncmp(token, CommandList[i].command, MEM_LEN) == 0){
					(*CommandList[i].function)();
					matched=true; 
					break;
				}
			}
			if (!matched && defaultHandlerExists) {
				(*defaultHandler)(errorCache); 
			}
		}
	}

	char *next(){
		char *nextToken;
		nextToken = strtok_r(NULL, delim, &last); 
		return nextToken; 
	}

	void addCommand(const char *command, void (*function)()){
		if (numCommand < MAXSERIALCOMMANDS) {		
			strncpy(CommandList[numCommand].command, command, MEM_LEN); 
			CommandList[numCommand].function = function; 
			numCommand++; 
		}
	}

	void addDefaultHandler(void (*function)(char*)){
		defaultHandler = function;
		defaultHandlerExists = true;
	}
};
#endif