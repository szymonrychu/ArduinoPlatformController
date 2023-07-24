#ifndef COMMAND_H
#define COMMAND_H
#include <string.h>

#ifndef MEM_LEN
#define MEM_LEN 256
#endif
#define MAXSERIALCOMMANDS	10
#define MAXDELIMETER 2

class JSONCommand{
private:
	char *receiveCache;
	char *last;
	char *token;

	void (*handler)(char*);

protected:
	int  bufPos;
	char buffer[MEM_LEN];
	bool termFound = false;
	char *term;
public:

	JSONCommand(char *terminator, void (*function)(char*)){
		handler = function;
		term = terminator;
		memset(buffer, 0, MEM_LEN);
	}


	void inputChar(char inChar){
		if(13 == (int)inChar)return; //discard carriage returns
		if(inChar == term[0]) {
			termFound = true;
			receiveCache = (char*)calloc(strlen(&(buffer[0]))+1, sizeof(char));
			strcpy(receiveCache, &(buffer[0]));
			bufPos = 0;
			memset(buffer, 0, MEM_LEN);
		}else{
			buffer[bufPos++]=inChar;
			buffer[bufPos]='\0';
			if (bufPos > MEM_LEN-1) bufPos=0;
		}
	}

	void inputString(char* data){
		for(uint16_t c=0; c<strlen(data); c++){
				inputChar(data[c]);
		}
	}

	void parse(){
		if(termFound){
			termFound = false;
			(*handler)(receiveCache);
		}
	}
};
#endif