/*
 * usbMsg.h
 *
 *  Created on: Oct 17, 2023
 *      Author: hasan
 */

#ifndef INC_USBMSG_H_
#define INC_USBMSG_H_


// MESSAGES TO COME
const char ARE_YOU_OK_MSG[4]        = {'$', '?', '\n',};
const char CAN_A_VAL_MSG[7]         = {'$', 'C', 'A', 'N', 'A','V', ','};
const char CAN_B_VAL_MSG[7]         = {'$', 'C', 'A', 'N', 'B','V', ','};
const char FILTER_ID_CAN_A_MSG[9]   = {'$', 'F', 'L', 'T', 'R', 'A', 'I', 'D', ','};
const char FILTER_ID_CAN_B_MSG[9]   = {'$', 'F', 'L', 'T', 'R', 'B', 'I', 'D', ','};
const char ROUTE_1_MSG[6]           = {'$', 'R', 'O', 'N', 'E', ','};
const char ROUTE_2_MSG[6]           = {'$', 'R', 'T', 'W', 'O', ','};
const char RESET_MSG[7]             = {'$', 'R', 'E', 'S', 'E', 'T', '\n'};

// MESSAGE TO SEND
const char FAIL_MSG[6]         = "$FAIL\n";
const char OK_MSG[4]           = "$OK\n";

const char TERMINATOR_OF_MSG   = '\n';

#endif /* INC_USBMSG_H_ */
