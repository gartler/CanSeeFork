/**
* @file utils.cpp
* @brief Utility functions.
*/

#include "utils.h"

/**
 * byte to hex String, variable string length
 * @param num Input
 * @returns Hex String
 */
String getHexSimple(uint8_t num)
{
	String stringOne = String(num, HEX);
	return stringOne;
}

/**
 * byte to hex String, variable but always even string length
 * @param num Input
 * @returns Hex String
 */
String getHex(uint32_t num)
{
	String stringOne = String(num, HEX);
	if (stringOne.length() < 2)
		stringOne = "0" + stringOne;
	return stringOne;
}

/**
 * Hex String to uint32_t, case insensitive input. Non 0-9, a-z characters
 * are ignored.
 * @param hexString Input
 * @returns uint32_t
 */
uint32_t hexToDec(String hexString)
{
	uint32_t decValue = 0;
	int nextInt;

	for (int i = 0; i < hexString.length(); i++)
	{
		nextInt = int(hexString.charAt(i));
		if (nextInt >= 48 && nextInt <= 57)
			nextInt = map(nextInt, 48, 57, 0, 9);
		else if (nextInt >= 65 && nextInt <= 70)
			nextInt = map(nextInt, 65, 70, 10, 15);
		else if (nextInt >= 97 && nextInt <= 102)
			nextInt = map(nextInt, 97, 102, 10, 15);
		else
			continue;
		nextInt = constrain(nextInt, 0, 15);
		decValue = (decValue * 16) + nextInt;
	}
	return decValue;
}
