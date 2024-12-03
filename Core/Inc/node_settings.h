/*
 * node_settings.h
 *
 *  Created on: Jul 5, 2024
 *      Author: warg
 */

#ifndef INC_NODE_SETTINGS_H_
#define INC_NODE_SETTINGS_H_

#define NODE_ID 1 // 0 if anonymous, otherwise between 1 and 127

/* Determines which ESC is controlled by flight controller.
 *
 * E.g. for a 4-rotor setup, we should set ESC_INDEX to 0, 1, 2, 3 for each
 * CAN ESC adapter board respectively.
 * 
 * ESC_INDEX is currently set to 0, this means the current configuration
 * is for the 1st CAN ESC adapter board attached to the flight controller.
 */
define ESC_INDEX 0 // between 0 and 32

#endif /* INC_NODE_SETTINGS_H_ */
