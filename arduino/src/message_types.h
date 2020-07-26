/*
 * message_types.h
 *
 * Created: 14.07.2020 12:46:58
 *  Author: xeedn
 */ 


#ifndef MESSAGE_TYPES_H_
#define MESSAGE_TYPES_H_

// Outgoing
#define MSG_INIT (1)
#define MSG_ALIVE (2)

// Incoming
#define MSG_START (42)
#define MSG_STOP (43)
#define MSG_THRUST (44)

#define MSG_LOG_ORIENTATION (45)
#define MSG_LOG_PID (46)
#define MSG_LOG_THRUST (47)

#define MSG_CONTROL (48)



#endif /* MESSAGE_TYPES_H_ */