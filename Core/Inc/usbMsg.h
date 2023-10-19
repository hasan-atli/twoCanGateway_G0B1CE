/*
 * usbMsg.h
 *
 *  Created on: Oct 17, 2023
 *      Author: hasan
 */

#ifndef INC_USBMSG_H_
#define INC_USBMSG_H_


// MESSAGES TO COME
const char ARE_YOU_OK_MSG[4]     = {'$', '?', '*',};
const char CAN_VAL_MSG[6]        = {'$', 'C', 'A', 'N', 'V', ','};
const char ROUTE_1_MSG[6]        = {'$', 'R', 'O', 'N', 'E', ','};
const char ROUTE_2_MSG[6]        = {'$', 'R', 'T', 'W', 'O', ','};


// MESSAGE TO SEND
const char FAIL_MSG[6] = "$FAIL*";
const char OK_MSG[4]   = "$OK*";

const char TERMINATOR_OF_MSG[]   = {'*'};

#endif /* INC_USBMSG_H_ */
