/*
Copyright 2015 - 2017 Andreas Chaitidis Andreas.Chaitidis@gmail.com

This program is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f7xx_hal.h"
#include "VescUart.h"
#include "buffer.h"
#include "crc.h"
#include <string.h>
#include <local_datatypes.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;



int ReceiveUartMessage(uint8_t* payloadReceived) {

	//Messages <= 255 start with 2. 2nd byte is length
	//Messages >255 start with 3. 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF
	 
	int counter = 0;
	int endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[256];
	int lenPayload = 0;

	while (1) {

		//messageReceived[counter++] = SERIALIO.read(); //	HAL_UART_Recieve(&huart2,messageReceived,1,10); counter++; �ϳ��� �о�

		if (counter == 2) {//case if state of 'counter' with last read 1

			switch (messageReceived[0])
			{
			case 2:
				endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
				lenPayload = messageReceived[1];
				break;
			case 3:
				//ToDo: Add Message Handling > 255 (starting with 3)
				break;
			default:
				break;
			}

		}
		if (counter >= sizeof(messageReceived))
		{
			break;
		}

		if (counter == endMessage && messageReceived[endMessage - 1] == 3) {//+1: Because of counter++ state of 'counter' with last read = "endMessage"
			messageReceived[endMessage] = 0;
#ifdef DEBUG
			DEBUGSERIAL.println("End of message reached!");
#endif			
			messageRead = true;
			break; //Exit if end of message is reached, even if there is still more data in buffer. 
		}
	}
	bool unpacked = false;
	if (messageRead) {
		unpacked = UnpackPayload(messageReceived, endMessage, payloadReceived, messageReceived[1]);
	}
	if (unpacked)
	{
		return lenPayload; //Message was read

	}
	else {
		return 0; //No Message Read
	}
}

bool UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPay) {
	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;
	//Rebuild src:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];
#ifdef DEBUG
	DEBUGSERIAL.print("SRC received: "); DEBUGSERIAL.println(crcMessage);
#endif // DEBUG

	//Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);
#ifdef DEBUG
	DEBUGSERIAL.print("SRC calc: "); DEBUGSERIAL.println(crcPayload);
#endif
	if (crcPayload == crcMessage)
	{
#ifdef DEBUG
		DEBUGSERIAL.print("Received: "); SerialPrint(message, lenMes); DEBUGSERIAL.println();
		DEBUGSERIAL.print("Payload :      "); SerialPrint(payload, message[1] - 1); DEBUGSERIAL.println();
#endif // DEBUG

		return true;
	}
	else
	{
		return false;
	}
}

int PackSendPayload(uint8_t* payload, int lenPay) {
	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}
	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	//messageSend[count] = NULL;

/*#ifdef DEBUG
	DEBUGSERIAL.print("UART package send: "); SerialPrint(messageSend, count);

#endif // DEBUG
*/
	//Sending package
	HAL_UART_Transmit(&huart3, messageSend, count,10);


	//Returns number of send bytes
	return count;
}


bool ProcessReadPacket(uint8_t* message, bldcMeasure *hvesc, int len) {
	COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++;//Eliminates the message id
	len--;

	switch (packetId)
	{
		case COMM_GET_VALUES:
			ind = 14; //Skipped the first 14 bit.
			hvesc->avgMotorCurrent = buffer_get_float32(message, 100.0, &ind);
			hvesc->avgInputCurrent = buffer_get_float32(message, 100.0, &ind);
			hvesc->dutyCycleNow = buffer_get_float16(message, 1000.0, &ind);
			hvesc->rpm = buffer_get_int32(message, &ind);
			hvesc->inpVoltage = buffer_get_float16(message, 10.0, &ind);
			hvesc->ampHours = buffer_get_float32(message, 10000.0, &ind);
			hvesc->ampHoursCharged = buffer_get_float32(message, 10000.0, &ind);
			ind += 8; //Skip 9 bit
			hvesc->tachometer = buffer_get_int32(message, &ind);
			hvesc->tachometerAbs = buffer_get_int32(message, &ind);
			return true;
			break;
			

		default:
			return false;
			break;
	}

}


bool VescUartGetValue(bldcMeasure *hvesc) {
	uint8_t command[1] = { COMM_GET_VALUES };
	uint8_t payload[256];
	PackSendPayload(command, 1);
	HAL_Delay(100); //needed, otherwise data is not read
	int lenPayload = ReceiveUartMessage(payload);
	if (lenPayload > 55) {
		bool read = ProcessReadPacket(payload, hvesc, lenPayload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

void VescUartSetDutyCycle(float dutyCycle) {
	int32_t index = 0;
	uint8_t payload[5];
	
	payload[index++] = COMM_SET_DUTY;
	buffer_append_int32(payload, (int32_t)(dutyCycle*100000), &index);
	PackSendPayload(payload, 5);
}

void VescUartSetCurrent(float current) {
	int32_t index = 0;
	uint8_t payload[5];
		
	payload[index++] = COMM_SET_CURRENT ;
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	PackSendPayload(payload, 5);
}

void VescUartSetCurrentBrake(float brakeCurrent) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);
	PackSendPayload(payload, 5);

}

/*
void VescUartSetNunchukValues(remotePackage& data) {
	int32_t ind = 0;
	uint8_t payload[11];
	payload[ind++] = COMM_SET_CHUCK_DATA;
	payload[ind++] = data.valXJoy;
	payload[ind++] = data.valYJoy;
	buffer_append_bool(payload, data.valLowerButton, &ind);
	buffer_append_bool(payload, data.valUpperButton, &ind);
	//Acceleration Data. Not used, Int16 (2 byte)
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;

#ifdef DEBUG
	DEBUGSERIAL.println("Data reached at VescUartSetNunchuckValues:");
	DEBUGSERIAL.print("valXJoy = "); DEBUGSERIAL.print(data.valXJoy); DEBUGSERIAL.print(" valYJoy = "); DEBUGSERIAL.println(data.valYJoy);
	DEBUGSERIAL.print("LowerButton = "); DEBUGSERIAL.print(data.valLowerButton); DEBUGSERIAL.print(" UpperButton = "); DEBUGSERIAL.println(data.valUpperButton);
#endif

	PackSendPayload(payload, 11);
}

void SerialPrint(uint8_t* data, int len) {

	//	DEBUGSERIAL.print("Data to display: "); DEBUGSERIAL.println(sizeof(data));

	for (int i = 0; i <= len; i++)
	{
		DEBUGSERIAL.print(data[i]);
		DEBUGSERIAL.print(" ");
	}
	DEBUGSERIAL.println("");
}


void SerialPrint(const bldcMeasure *hvesc) {
	DEBUGSERIAL.print("avgMotorCurrent: "); DEBUGSERIAL.println(hvesc->avgMotorCurrent);
	DEBUGSERIAL.print("avgInputCurrent: "); DEBUGSERIAL.println(hvesc->avgInputCurrent);
	DEBUGSERIAL.print("dutyCycleNow: "); DEBUGSERIAL.println(hvesc->dutyCycleNow);
	DEBUGSERIAL.print("rpm: "); DEBUGSERIAL.println(hvesc->rpm);
	DEBUGSERIAL.print("inputVoltage: "); DEBUGSERIAL.println(hvesc->inpVoltage);
	DEBUGSERIAL.print("ampHours: "); DEBUGSERIAL.println(hvesc->ampHours);
	DEBUGSERIAL.print("ampHoursCharges: "); DEBUGSERIAL.println(hvesc->ampHoursCharged);
	DEBUGSERIAL.print("tachometer: "); DEBUGSERIAL.println(hvesc->tachometer);
	DEBUGSERIAL.print("tachometerAbs: "); DEBUGSERIAL.println(hvesc->tachometerAbs);
}
*/