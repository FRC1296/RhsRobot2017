/** \file
 * Tokens used in our scripting language
 */

#ifndef AUTOPARSER_H
#define AUTOPARSER_H

// any line in the parser file that begins with a space or a # is skipped

const char sComment = '#';
const char szDelimiters[] = " ,[]()\r\n\t";

///N - doesn't need a response; R - needs a response; _ - contained within auto thread
typedef enum AUTO_COMMAND_TOKENS
{
	AUTO_TOKEN_MODE,				//!<	mode block number, number(integer)
	AUTO_TOKEN_DEBUG,				//!<	debug mode, 0 = off, 1 = on
	AUTO_TOKEN_MESSAGE,				//!<	print debug message
	AUTO_TOKEN_BEGIN,				//!<	mark beginning of mode block
	AUTO_TOKEN_END,					//!<	mark end of mode block
	AUTO_TOKEN_DELAY,				//!<	delay (seconds - float)
	AUTO_TOKEN_MOVE,				//!<N	move (left & right PWM - float)
	AUTO_TOKEN_MMOVE,				//!<R	mmove <speed> (inches - float)
	AUTO_TOKEN_MPROXIMITY,	        //!<R	mprox <speed> (inches - float)
	AUTO_TOKEN_TURN,				//!<R	turn <degrees - float> (timeout)
	AUTO_TOKEN_GEAR_RELEASE, 		//!<    opens gear handler
	AUTO_TOKEN_GEAR_HOLD,			//!< 	closes gear handler
	AUTO_TOKEN_GEAR_HANG,			//!< 	gear hang macro
	AUTO_TOKEN_CLIMBER,			    //!< 	moves climber a bit

	AUTO_TOKEN_LAST
} AUTO_COMMAND_TOKENS;

#endif  // AUTOPARSER_H

