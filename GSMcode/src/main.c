/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>
#include <drivers/uart.h>
#include <data/json.h>
#define apn "airtelgprs.com"

u8_t size[] = "+CIPRXGET: 3,1,23,0";

//Connection to GPRS command block start
// Set the Bearer for the IP
u8_t apn1[] = "AT+SAPBR=3,1,\"APN\",\"";
u8_t apn2[] = "airtelgprs.com";
u8_t apn3[] = "\n";
u8_t setConnectionType[] = "AT+SAPBR=3,1,\"Contype\",\"GPRS\"\n";	   // Set the connection type to GPRS
u8_t apnConnect[] = "AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"\n";	   // Set the APN
u8_t username[] = "AT+SAPBR=3,1,\"USER\",\"NULL\"\n";				   // Set the user name
u8_t passwordSet[] = "AT+SAPBR=3,1,\"PWD\",\"NULL\"\n";				   // Set the password
u8_t definePdpContext[] = "AT+CGDCONT=1,\"IP\",\"airtelgprs.com\"\n";  // Define the PDP context
u8_t connectPdp[] = "AT+CGACT=1,1\n";								   // Activate the PDP context
u8_t openDefinedGprsContext[] = "AT+SAPBR=1,1\n";					   // Open the definied GPRS bearer context
u8_t queryGprsContext[] = "AT+SAPBR=2,1\n";							   // Query the GPRS bearer context status
u8_t connectGprs[] = "AT+CGATT=1\n";								   // Attach to GPRS
u8_t setMultiIP[] = "AT+CIPMUX=1\n";								   // Set to multi-IP
u8_t quickSend[] = "AT+CIPQSEND=1\n";								   // Put in "quick send" mode (thus no extra "Send OK")
u8_t setGetDataManual[] = "AT+CIPRXGET=1\n";						   // Set to get data manually
u8_t startTask[] = {"AT+CSTT=\"airtelgprs.com\",\"NULL\",\"NULL\"\n"}; // Start Task and Set APN, USER NAME, PASSWORD
u8_t startConnection[] = "AT+CIICR\n";								   // Bring Up Wireless Connection with GPRS or CSD
u8_t getLocalIP[] = "AT+CIFSR;E0\n";								   // Get Local IP Address, only assigned after connection
u8_t configureDNS[] = "AT+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\"\n";		   // Configure Domain Name Server (DNS)
// Connection to GPRS commands block ends

#define MQTTCONNECT 1 << 4		// Client request to connect to Server
#define MQTTCONNACK 2 << 4		// Connect Acknowledgment
#define MQTTPUBLISH 3 << 4		// Publish message
#define MQTTPUBACK 4 << 4		// Publish Acknowledgment
#define MQTTPUBREC 5 << 4		// Publish Received (assured delivery part 1)
#define MQTTPUBREL 6 << 4		// Publish Release (assured delivery part 2)
#define MQTTPUBCOMP 7 << 4		// Publish Complete (assured delivery part 3)
#define MQTTSUBSCRIBE 8 << 4	// Client Subscribe request
#define MQTTSUBACK 9 << 4		// Subscribe Acknowledgment
#define MQTTUNSUBSCRIBE 10 << 4 // Client Unsubscribe request
#define MQTTUNSUBACK 11 << 4	// Unsubscribe Acknowledgment
#define MQTTPINGREQ 12 << 4		// PING Request
#define MQTTPINGRESP 13 << 4	// PING Response
#define MQTTDISCONNECT 14 << 4	// Client is Disconnecting
#define MQTTReserved 15 << 4	// Reserved

u8_t serverConnect[] = "AT+CIPSTART=1,\"TCP\",\"test.mosquitto.org\",1883\n";
u8_t connectPacket[] = {0x10, 0x14, 0x00, 0x04, 0X4D, 0X51, 0X54, 0X54, 0x04, 0x02, 0x00, 0x3c, 0x00, 0x08, 0x61, 0x78, 0x64, 0x65, 0x73, 0x69, 0x67, 0x6e};
u8_t publishPacket[] = {0x30, 0xF, 0x00, 0x08, 0x61, 0x78, 0x64, 0x65, 0x73, 0x69, 0x67, 0x6e, 0x00, 0x03, 0x68, 0x65, 0x79};
u8_t subscribePacket[] = {0x82, 0xD, 0x00, 0x01, 0x00, 0x08, 0x61, 0x78, 0x64, 0x65, 0x73, 0x69, 0x67, 0x6e, 0x00};
u8_t puback[100] = {0};
u8_t puback1[100] = {0};
u8_t puback2[100] = {0};
u8_t suback[100] = {0};
u8_t suback1[100] = {0};
u8_t suback2[100] = {0};
u8_t receiveConnack[] = "AT+CIPRXGET=3,1,200\n";
u8_t connack[100] = {0};
u8_t connack1[100] = {0};
u8_t connack2[100] = {0};
u8_t response[100] = {0};
u8_t response1[50] = {0};
u8_t response2[100] = {0};
u8_t response3[100] = {0};
u8_t response4[100] = {0};
u8_t response5[100] = {0};
u8_t response6[100] = {0};
u8_t response7[100] = {0};
u8_t response8[100] = {0};
u8_t response9[100] = {0};
u8_t response10[100] = {0};
u8_t response11[100] = {0};
u8_t response12[100] = {0};
u8_t response13[100] = {0};
u8_t response14[100] = {0};
u8_t response15[100] = {0};
u8_t response16[100] = {0};
u8_t hello[] = "Hey";
u8_t hello1[] = " hello1";
u8_t there[50] = { 0 };

//Command packets to connect to MQTT
u8_t sendCon[] = "AT+CIPSEND=1,22\n";
u8_t sendPub[] = "AT+CIPSEND=1,17\n";
u8_t sendSub[] = "AT+CIPSEND=1,15\n";
u8_t status[] = "AT+CIPSTATUS=1\n";
u8_t enter[] = "\n";

int i = 0;
int b = 0;

#define SLEEP_TIME_MS 1

#ifndef DT_ALIAS_SW0_GPIOS_FLAGS
#define DT_ALIAS_SW0_GPIOS_FLAGS 0
#endif

#ifndef DT_ALIAS_LED0_GPIOS_FLAGS
#define DT_ALIAS_LED0_GPIOS_FLAGS 0
#endif

#define SENDAT "AT"
int state = 0;
int oddEven = 0;
int buttonFlag = 0;

void button_pressed(struct device *dev, struct gpio_callback *cb, u32_t pins)
{
	// printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
	buttonFlag++;
	state = 1;
}

u8_t hex_to_int(struct device *dev, u8_t c)
{
	dev = device_get_binding("UART_0");
	u8_t tempString1[] = "= first\n";
	u8_t tempString2[] = "= second\n";
	u8_t tempString5[] = "= hexToInt\n";
	uart_tx(dev, tempString5, sizeof(tempString5), 50);
	k_sleep(50);

	if (c >= 97)
		c = c - 32;
	u8_t first = c / 16 - 3;
	k_sleep(50);
	u8_t first1[] = {0X61};

	uart_tx(dev, first1, sizeof(first1), 50);
	// uart_tx(dev, first, sizeof(first), 50);
	k_sleep(50);
	uart_tx(dev, tempString1, sizeof(tempString1), 50);
	k_sleep(50);
	u8_t second = c % 16;
	uart_tx(dev, second, sizeof(second), 50);
	k_sleep(50);
	uart_tx(dev, tempString2, sizeof(tempString2), 50);
	k_sleep(50);
	u8_t result = first * 10 + second;
	if (result > 9)
		result--;
	return result;
}

u8_t hex_to_ascii(struct device *dev, char c, char d)
{
	dev = device_get_binding("UART_0");
	u8_t tempString3[] = " = high\n";
	u8_t tempString4[] = " = low\n";
	u8_t high = hex_to_int(dev, c) * 16;
	k_sleep(10);
	uart_tx(dev, high, sizeof(high), 50);
	k_sleep(10);
	uart_tx(dev, tempString3, sizeof(tempString3), 50);
	k_sleep(10);
	u8_t low = hex_to_int(dev, d);
	uart_tx(dev, low, sizeof(low), 50);
	k_sleep(10);
	uart_tx(dev, tempString4, sizeof(tempString4), 50);
	k_sleep(10);
	return high + low;
}

int connectGPRS(struct device *dev, struct device *serial)
{
	uart_tx(dev, setConnectionType, sizeof(setConnectionType), 50);
	k_sleep(100);
	uart_rx_enable(dev, response, sizeof(response), 50);
	uart_tx(serial, response, sizeof(response), 50);
	k_sleep(100);

	uart_tx(dev, apnConnect, sizeof(apnConnect), 50);
	k_sleep(100);
	uart_rx_enable(dev, response1, sizeof(response1), 50);
	uart_tx(serial, response1, sizeof(response1), 50);
	k_sleep(100);

	uart_tx(dev, username, sizeof(username), 50);
	k_sleep(100);
	uart_rx_enable(dev, response2, sizeof(response2), 50);
	uart_tx(serial, response2, sizeof(response2), 50);
	k_sleep(100);

	uart_tx(dev, passwordSet, sizeof(passwordSet), 50);
	k_sleep(100);
	uart_rx_enable(dev, response3, sizeof(response3), 50);
	uart_tx(serial, response3, sizeof(response3), 50);
	k_sleep(100);

	uart_tx(dev, definePdpContext, sizeof(definePdpContext), 50);
	k_sleep(100);
	uart_rx_enable(dev, response4, sizeof(response4), 50);
	uart_tx(serial, response4, sizeof(response4), 50);
	k_sleep(600);

	uart_tx(dev, connectPdp, sizeof(connectPdp), 50);
	k_sleep(100);
	uart_rx_enable(dev, response5, sizeof(response5), 50);
	uart_tx(serial, response5, sizeof(response5), 50);
	k_sleep(850);

	uart_tx(dev, openDefinedGprsContext, sizeof(openDefinedGprsContext), 50);
	k_sleep(100);
	uart_rx_enable(dev, response6, sizeof(response6), 50);
	uart_tx(serial, response6, sizeof(response6), 50);
	k_sleep(3500);

	uart_tx(dev, queryGprsContext, sizeof(queryGprsContext), 50);
	k_sleep(100);
	uart_rx_enable(dev, response7, sizeof(response7), 50);
	uart_tx(serial, response7, sizeof(response7), 50);
	k_sleep(6000);

	uart_tx(dev, connectGprs, sizeof(setConnectionType), 50);
	k_sleep(100);
	uart_rx_enable(dev, response8, sizeof(response8), 50);
	uart_tx(serial, response8, sizeof(response8), 50);
	k_sleep(6000);

	uart_tx(dev, setMultiIP, sizeof(setMultiIP), 50);
	k_sleep(100);
	uart_rx_enable(dev, response9, sizeof(response9), 50);
	uart_tx(serial, response9, sizeof(response9), 50);
	k_sleep(1000);

	uart_tx(dev, quickSend, sizeof(quickSend), 50);
	k_sleep(100);
	uart_rx_enable(dev, response10, sizeof(response10), 50);
	uart_tx(serial, response10, sizeof(response10), 50);
	k_sleep(1000);

	uart_tx(dev, setGetDataManual, sizeof(setGetDataManual), 50);
	k_sleep(100);
	uart_rx_enable(dev, response11, sizeof(response11), 50);
	uart_tx(serial, response11, sizeof(response11), 50);
	k_sleep(1000);

	uart_tx(dev, startTask, sizeof(startTask), 50);
	k_sleep(100);
	uart_rx_enable(dev, response12, sizeof(response12), 50);
	uart_tx(serial, response12, sizeof(response12), 50);
	k_sleep(6000);

	uart_tx(dev, startConnection, sizeof(startConnection), 50);
	k_sleep(100);
	uart_rx_enable(dev, response13, sizeof(response13), 50);
	uart_tx(serial, response13, sizeof(response13), 50);
	k_sleep(6000);

	uart_tx(dev, getLocalIP, sizeof(getLocalIP), 50);
	k_sleep(100);
	uart_rx_enable(dev, response14, sizeof(response14), 50);
	uart_tx(serial, response14, sizeof(response14), 50);
	k_sleep(1000);

	// uart_tx(dev, configureDNS, sizeof(configureDNS), 50);
	// k_sleep(100);
	// uart_rx_enable(dev, response15, sizeof(response15), 50);
	// uart_tx(serial, response15, sizeof(response15), 50);
	// k_sleep(1000);

	uart_tx(dev, serverConnect, sizeof(serverConnect), 50);
	k_sleep(2000);
	uart_rx_enable(dev, response16, sizeof(response16), 50);
	uart_tx(serial, response16, sizeof(response16), 50);
	k_sleep(2000);

	// return 0;
}

int sendConnect(struct device *dev, struct device *serial)
{
	uart_tx(dev, status, sizeof(status), 50);
	uart_rx_enable(dev, response1, sizeof(response1), 50);
	uart_tx(serial, response1, sizeof(response1), 50);

	k_sleep(500);

	uart_tx(dev, sendCon, sizeof(sendCon), 50);
	uart_rx_enable(dev, response1, sizeof(response1), 50);
	uart_tx(serial, response1, sizeof(response1), 50);

	// k_sleep(500);

	uart_tx(dev, connectPacket, sizeof(connectPacket), 50);
	uart_tx(dev, enter, sizeof(enter), 50);
	uart_rx_enable(dev, connack, sizeof(connack), 50);
	uart_tx(serial, connack, sizeof(connack), 50);

	k_sleep(500);

	uart_tx(dev, receiveConnack, sizeof(receiveConnack), 50);
	uart_rx_enable(dev, connack1, sizeof(connack1), 50);
	uart_tx(serial, connack1, sizeof(connack1), 50);

	uart_rx_enable(dev, connack2, sizeof(connack2), 50);
	uart_tx(serial, connack2, sizeof(connack2), 50);
	k_sleep(500);
}

int sendPublish(struct device *dev, struct device *serial)
{
}

static struct gpio_callback button_cb_data;

int mqtt_callback(struct device *uartGsm, struct device *uartSerial)
{
	u8_t subs[100] = {0};
	u8_t response[50] = {0};
	uart_tx(uartGsm, receiveConnack, sizeof(receiveConnack), 50);
	uart_rx_enable(uartGsm, subs, sizeof(subs), 50);
	uart_rx_enable(uartGsm, response, sizeof(response), 50);
	uart_tx(uartSerial, subs, sizeof(subs), 50);
	uart_tx(uartSerial, response, sizeof(response), 50);
}

void main(void)

{

	struct device *devButton, *uartGsm, *uartSerial;
	int ret;

	u8_t info[] = "ATI\n";
	u8_t test[] = "AT\n";
	u8_t passwordRequired[] = "AT+CPIN?\n";
	u8_t signalStrength[] = "AT+CSQ\n";
	u8_t registerationME[10] = "AT+CREG?\n";
	// u8_t gprsStatus[] = "AT+CGREG?\n";
	u8_t gprsStatus[] = "AT+CGATT?\n";

	// u8_t response1[15] = {0};
	u8_t echoOn[] = "ATE1\n";

	// uart_tx(uartGsm, echoOn, sizeof(echoOn), 50);
	// uart_rx_enable(uartGsm, response1, sizeof(response1), 50);
	// uart_tx(uartSerial, response1, sizeof(response1), 50);

	devButton = device_get_binding(DT_ALIAS_SW0_GPIOS_CONTROLLER);
	if (devButton == NULL)
	{
		printk("Error: didn't find %s device\n",
			   DT_ALIAS_SW0_GPIOS_CONTROLLER);
		return;
	}

	uartGsm = device_get_binding("UART_1");
	uartSerial = device_get_binding("UART_0");

	ret = gpio_pin_configure(devButton, DT_ALIAS_SW0_GPIOS_PIN,
							 DT_ALIAS_SW0_GPIOS_FLAGS | GPIO_INPUT);
	if (ret != 0)
	{
		printk("Error %d: failed to configure pin %d '%s'\n", ret,
			   DT_ALIAS_SW0_GPIOS_PIN, DT_ALIAS_SW0_LABEL);
		return;
	}

	ret = gpio_pin_interrupt_configure(devButton, DT_ALIAS_SW0_GPIOS_PIN,
									   GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0)
	{
		printk("Error %d: failed to configure interrupt on pin %d '%s'\n",
			   ret, DT_ALIAS_SW0_GPIOS_PIN, DT_ALIAS_SW0_LABEL);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed,
					   BIT(DT_ALIAS_SW0_GPIOS_PIN));
	gpio_add_callback(devButton, &button_cb_data);

	// u8_t response[1000] = {0};

	u8_t response[50] = {0};
	u8_t response1[50] = {0};
	u8_t response2[50] = {0};
	u8_t response3[50] = {0};
	u8_t response4[50] = {0};
	u8_t response5[50] = {0};
	u8_t response6[50] = {0};
	u8_t response7[50] = {0};
	u8_t response8[50] = {0};
	u8_t response9[50] = {0};
	u8_t response10[50] = {0};
	u8_t response11[50] = {0};
	u8_t response12[50] = {0};
	u8_t response13[50] = {0};
	u8_t response14[50] = {0};
	u8_t response15[50] = {0};
	u8_t response16[50] = {0};
	char *asc[50] = {0};
	uart_tx(uartSerial, enter, sizeof(enter), 50);
	u8_t clear123[] = "Hello\n";
	k_sleep(10);
	uart_tx(uartSerial, clear123, sizeof(clear123), 50);
	k_sleep(10);
	uart_tx(uartSerial, enter, sizeof(enter), 50);
	k_sleep(10);

	u8_t s[] = "61";
	int tempVariable1[] = {61};
	char tempVariable2[] = "61";
	// uart_tx(uartSerial, s, sizeof(s), 50);
	k_sleep(500);
	uart_tx(uartSerial, tempVariable1, sizeof(tempVariable1), 50);
	k_sleep(500);
	// uart_tx(uartSerial, tempVariable2, sizeof(tempVariable2), 50);
	k_sleep(500);

	int lengt = sizeof(s) - 1;
	int h;
	u8_t bu = 0;
	for (h = 0; h < lengt; h++)
	{
		if (h % 2 != 0)
		{
			u8_t as = hex_to_ascii(uartGsm, bu, s[h]);
			char *ascii = as;
			uart_tx(uartSerial, ascii, sizeof(ascii), 50);
			k_sleep(500);
			uart_tx(uartSerial, as, sizeof(as), 50);
			// printk("%c", hex_to_ascii(bu, s[h]));
		}
		else
		{
			bu = s[h];
		}
	}

#ifdef DT_ALIAS_LED0_GPIOS_CONTROLLER
	struct device *dev_led;

	dev_led = device_get_binding(DT_ALIAS_LED0_GPIOS_CONTROLLER);
	if (dev_led == NULL)
	{
		printk("Error: didn't find %s device\n",
			   DT_ALIAS_LED0_GPIOS_CONTROLLER);
		return;
	}

	ret = gpio_pin_configure(dev_led, DT_ALIAS_LED0_GPIOS_PIN,
							 DT_ALIAS_LED0_GPIOS_FLAGS | GPIO_OUTPUT);
	if (ret != 0)
	{
		printk("Error %d: failed to configure pin %d '%s'\n", ret,
			   DT_ALIAS_LED0_GPIOS_PIN, DT_ALIAS_LED0_LABEL);
		return;
	}

#endif
	// printk("Press %s on the board\n", DT_ALIAS_SW0_LABEL);

	while (1)
	{
#ifdef DT_ALIAS_LED0_GPIOS_CONTROLLER
		bool val;

		val = gpio_pin_get(devButton, DT_ALIAS_SW0_GPIOS_PIN);
		gpio_pin_set(dev_led, DT_ALIAS_LED0_GPIOS_PIN, val);
		k_sleep(SLEEP_TIME_MS);

		oddEven = buttonFlag % 2;
		if (state == 1 && oddEven == 1)
		{
			u8_t command1[] = "\nSending function commands\n";
			uart_tx(uartSerial, command1, sizeof(command1), 100);
			connectGPRS(uartGsm, uartSerial);

			// u8_t command[] = "Sending commands after button press\n";

			// uart_tx(uartSerial, command, sizeof(command), 100);
			// connectGPRS(uartGsm, uartSerial);

			// uart_tx(uartGsm, setConnectionType, sizeof(setConnectionType), 50);
			// k_sleep(100);
			// uart_rx_enable(uartGsm, response, sizeof(response), 50);
			// uart_tx(uartSerial, response, sizeof(response), 50);
			// k_sleep(100);

			// uart_tx(uartGsm, apnConnect, sizeof(apnConnect), 50);
			// uart_rx_enable(uartGsm, response, sizeof(response), 50);
			// uart_tx(uartSerial, response, sizeof(response), 50);
			// k_sleep(100);

			// uart_tx(uartGsm, username, sizeof(username), 50);
			// k_sleep(100);
			// uart_rx_enable(uartGsm, response3, sizeof(response3), 50);
			// uart_tx(uartSerial, response3, sizeof(response3), 50);
			// k_sleep(100);

			// uart_tx(dev, definePdpContext, sizeof(definePdpContext), 50);
			// k_sleep(100);
			// uart_rx_enable(dev, response4, sizeof(response4), 50);
			// uart_tx(serial, response4, sizeof(response4), 50);
			// k_sleep(600);

			// uart_tx(dev, connectPdp, sizeof(connectPdp), 50);
			// k_sleep(100);
			// uart_rx_enable(dev, response5, sizeof(response5), 50);
			// uart_tx(serial, response5, sizeof(response5), 50);
			// k_sleep(850);

			// uart_tx(uartGsm, registerationME, sizeof(registerationME), 50);
			// uart_rx_enable(uartGsm, response, sizeof(response), 50);
			// k_sleep(100);
			// uart_tx(uartSerial, response, sizeof(response), 50);
			// k_sleep(10);

			// uart_tx(uartGsm, gprsStatus, sizeof(gprsStatus), 50);
			// uart_rx_enable(uartGsm, response, sizeof(response), 50);
			// k_sleep(100);
			// uart_tx(uartSerial, response, sizeof(response), 50);
			// k_sleep(10);
			// uart_rx_disable(uartGsm);

			uart_tx(uartGsm, status, sizeof(status), 50);
			uart_rx_enable(uartGsm, response1, sizeof(response1), 50);
			uart_tx(uartSerial, response1, sizeof(response1), 50);

			k_sleep(500);

			uart_tx(uartGsm, sendCon, sizeof(sendCon), 50);
			uart_rx_enable(uartGsm, response1, sizeof(response1), 50);
			uart_tx(uartSerial, response1, sizeof(response1), 50);
			k_sleep(100);
			uart_tx(uartGsm, connectPacket, sizeof(connectPacket), 50);
			uart_tx(uartGsm, enter, sizeof(enter), 50);
			uart_rx_enable(uartSerial, connack, sizeof(connack), 50);
			uart_tx(uartSerial, connack, sizeof(connack), 50);
			state = 0;

			uart_tx(uartGsm, receiveConnack, sizeof(receiveConnack), 50);
			uart_rx_enable(uartGsm, connack1, sizeof(connack1), 50);
			uart_tx(uartSerial, connack1, sizeof(connack1), 50);

			uart_rx_enable(uartGsm, connack2, sizeof(connack2), 50);
			uart_tx(uartSerial, connack2, sizeof(connack2), 50);
			k_sleep(500);

			uart_tx(uartGsm, status, sizeof(status), 50);
			uart_rx_enable(uartGsm, response1, sizeof(response1), 50);
			uart_tx(uartSerial, response1, sizeof(response1), 50);

			k_sleep(500);

			uart_tx(uartGsm, sendPub, sizeof(sendPub), 50);
			uart_rx_enable(uartGsm, response1, sizeof(response1), 50);
			uart_tx(uartSerial, response1, sizeof(response1), 50);
			k_sleep(100);
			uart_tx(uartGsm, publishPacket, sizeof(publishPacket), 50);
			uart_tx(uartGsm, enter, sizeof(enter), 50);
			uart_rx_enable(uartSerial, puback, sizeof(puback), 50);
			uart_tx(uartSerial, puback1, sizeof(puback1), 50);
			state = 0;

			uart_tx(uartGsm, receiveConnack, sizeof(receiveConnack), 50);
			uart_rx_enable(uartGsm, connack1, sizeof(connack1), 50);
			uart_tx(uartSerial, connack1, sizeof(connack1), 50);

			uart_rx_enable(uartGsm, puback2, sizeof(puback2), 50);
			uart_tx(uartSerial, puback2, sizeof(puback2), 50);
			k_sleep(500);

			while (connack1[0] != '+')
			{
				uart_tx(uartGsm, status, sizeof(status), 50);
				uart_rx_enable(uartGsm, response1, sizeof(response1), 50);
				uart_tx(uartSerial, response1, sizeof(response1), 50);

				k_sleep(500);

				uart_tx(uartGsm, receiveConnack, sizeof(receiveConnack), 50);
				uart_rx_enable(uartGsm, connack1, sizeof(connack1), 50);
				uart_tx(uartSerial, connack1, sizeof(connack1), 50);

				k_sleep(500);

				uart_rx_enable(uartGsm, response1, sizeof(response1), 50);
				uart_tx(uartSerial, response1, sizeof(response1), 50);
				k_sleep(100);

				break;
			}

			// uart_tx(uartGsm, sendSub, sizeof(sendSub), 50);
			// uart_rx_enable(uartGsm, suback, sizeof(suback), 50);
			// uart_tx(uartSerial, suback, sizeof(suback), 50);

			// uart_tx(uartGsm, subscribePacket, sizeof(subscribePacket), 50);
			// // uart_tx(uartGsm, enter, sizeof(enter), 50);
			// uart_rx_enable(uartGsm, suback, sizeof(suback), 50);
			// uart_tx(uartSerial, suback, sizeof(suback), 50);

			// uart_rx_enable(uartGsm, suback1, sizeof(suback1), 50);
			// uart_tx(uartSerial, suback1, sizeof(suback1), 50);

			// uart_rx_enable(uartGsm, suback2, sizeof(suback2), 50);
			// uart_tx(uartSerial, suback2, sizeof(suback2), 50);

			// uart_tx(uartGsm, subscribePacket, sizeof(subscribePacket), 50);
			// uart_tx(uartGsm, enter, sizeof(enter), 50);
			// uart_rx_enable(uartSerial, suback, sizeof(suback), 50);
			// uart_tx(uartSerial, suback1, sizeof(suback1), 50);
			// state = 0;

			// uart_tx(uartGsm, receiveConnack, sizeof(receiveConnack), 50);
			// uart_rx_enable(uartGsm, suback1, sizeof(suback1), 50);
			// uart_tx(uartSerial, suback1, sizeof(connack1), 50);

			// uart_rx_enable(uartGsm, suback2, sizeof(suback2), 50);
			// uart_tx(uartSerial, suback2, sizeof(puback2), 50);
			// k_sleep(500);

			// uart_tx(uartGsm, sendSub, sizeof(sendSub), 50);
			// k_sleep(100);
			// uart_tx(uartGsm, subscribePacket, sizeof(subscribePacket), 50);
			// uart_rx_enable(uartGsm, suback, sizeof(suback), 50);
			// // uart_tx(uartSerial, suback, sizeof(suback), 50);
			// uart_tx(uartGsm, status, sizeof(status), 50);
			// uart_rx_enable(uartGsm, response1, sizeof(response1), 50);
			// uart_tx(uartSerial, response1, sizeof(response1), 50);

			state = 0;
		}
		// k_sleep(100);

		if (state == 1 && oddEven == 0)
		{
			// u8_t command[] = "Sending commands after button press\n";

			// uart_tx(uartSerial, command, sizeof(command), 100);

			// sendConnect(uartGsm, uartSerial);

			uart_tx(uartGsm, status, sizeof(status), 50);
			uart_rx_enable(uartGsm, response1, sizeof(response1), 50);
			uart_tx(uartSerial, response1, sizeof(response1), 50);

			k_sleep(500);

			// uart_tx(uartGsm, sendCon, sizeof(sendCon), 50);
			// uart_rx_enable(uartGsm, response1, sizeof(response1), 50);
			// uart_tx(uartSerial, response1, sizeof(response1), 50);
			// k_sleep(100);
			// uart_tx(uartGsm, connectPacket, sizeof(connectPacket), 50);
			// uart_tx(uartGsm, enter, sizeof(enter), 50);
			// uart_rx_enable(uartSerial, connack, sizeof(connack), 50);
			// uart_tx(uartSerial, connack, sizeof(connack), 50);

			uart_tx(uartGsm, sendSub, sizeof(sendSub), 50);
			k_sleep(500);
			uart_tx(uartGsm, subscribePacket, sizeof(subscribePacket), 50);
			char subscription[50] = {0};

			while (b != 1)
			{
				// for (i = 0; i < sizeof(response1); i++)
				// {
				// 	response1[i] = 0;
				// 	subscription[i] = 0;
				// }

				uart_tx(uartGsm, status, sizeof(status), 50);
				uart_rx_enable(uartGsm, response1, sizeof(response1), 50);
				uart_tx(uartSerial, response1, sizeof(response1), 50);
				k_sleep(500);

				uart_tx(uartGsm, receiveConnack, sizeof(receiveConnack), 50);
				uart_rx_enable(uartGsm, subscription, sizeof(subscription), 50);
				uart_tx(uartSerial, subscription, sizeof(subscription), 50);

				for (i = 0; i < sizeof(response1); i++)
				{
					if (subscription[i] == '3' && subscription[i + 1] == ',' && subscription[i + 2] == '1' && subscription[i + 3] == ',' && subscription[i + 4] == '2' & subscription[i + 5] == '4' && subscription[i + 6] == ',')
					{
						for (int j = 0; j < 50; j++)
						{
							*asc[j] == subscription[i + j];
						}
						uart_tx(uartSerial, asc, sizeof(asc), 50);
						uart_tx(uartSerial, subscription, sizeof(subscription), 50);
						k_sleep(100);
						uart_rx_enable(uartGsm, response10, sizeof(response10), 50);
						uart_tx(uartSerial, response10, sizeof(response10), 50);
						u8_t broke[] = "Out of the while loop";
						uart_tx(uartSerial, broke, sizeof(broke), 50);

						k_sleep(1000);

						// char *st = "48656C6C6F3B";
						// *st = *asc;
						// int length = strlen(st);
						// int i;
						// char buf = 0;
						// for (i = 0; i < length; i++)
						// {
						// 	if (i % 2 != 0)
						// 	{
						// 		int as = hex_to_ascii(buf, st[i]);
						// 		char ascii = as;
						// 		uart_tx(uartSerial, ascii, sizeof(ascii), 50);
						// 		// printf("%c", hex_to_ascii(buf, st[i]));
						// 	}
						// 	else
						// 	{
						// 		buf = st[i];
						// 	}
						// }
						b = 1;
					}
				}
				k_sleep(500);
				// if (b == 1)
				// {
				// 	break;
				// }
			}
			u8_t broke[] = "Out of the while loop";
			uart_tx(uartSerial, broke, sizeof(broke), 50);

			// char *st = "301500086178646573696O";
			// *st = *asc;
			// int length = strlen(st);
			// int i;
			// char buf = 0;
			// for (i = 0; i < length; i++)
			// {
			// 	if (i % 2 != 0)
			// 	{
			// 		int as = hex_to_ascii(buf, st[i]);
			// 		char ascii = as;
			// 		uart_tx(uartSerial, ascii, sizeof(ascii), 50);
			// 		// printf("%c", hex_to_ascii(buf, st[i]));
			// 	}
			// 	else
			// 	{
			// 		buf = st[i];
			// 	}
			// }
			// uart_rx_enable(uartGsm, suback, sizeof(suback), 50);
			// // uart_tx(uartSerial, suback, sizeof(suback), 50);
			// uart_tx(uartGsm, status, sizeof(status), 50);
			// uart_rx_enable(uartGsm, response1, sizeof(response1), 50);
			// uart_tx(uartSerial, response1, sizeof(response1), 50);

			// uart_tx(uartGsm, subscribePacket, sizeof(subscribePacket), 50);
			// uart_rx_enable(uartGsm, suback, sizeof(suback), 50);
			// uart_tx(uartSerial, suback, sizeof(suback), 50);
			uart_tx(uartGsm, status, sizeof(status), 50);
			uart_rx_enable(uartGsm, response1, sizeof(response1), 50);
			uart_tx(uartSerial, response1, sizeof(response1), 50);

			// while (1)
			// {
			// 	u8_t resp[50] = {0};
			// 	uart_tx(uartGsm, status, sizeof(status), 50);
			// 	uart_rx_enable(uartGsm, resp, sizeof(resp), 50);
			// 	uart_tx(uartSerial, resp, sizeof(resp), 50);
			// 	mqtt_callback(uartGsm, uartSerial);
			// }

			state = 0;

			// while (connack2[0] != '+')
			// {

			// 	uart_tx(uartGsm, status, sizeof(status), 50);
			// 	uart_rx_enable(uartGsm, response1, sizeof(response1), 50);
			// 	uart_tx(uartSerial, response1, sizeof(response1), 50);

			// 	// uart_tx(uartGsm, receiveConnack, sizeof(receiveConnack), 50);
			// 	// uart_rx_enable(uartGsm, connack1, sizeof(connack1), 50);
			// 	// uart_tx(uartSerial, connack1, sizeof(connack1), 50);

			// 	uart_rx_enable(uartGsm, puback, sizeof(puback), 50);
			// 	uart_tx(uartSerial, puback, sizeof(puback), 50);
			// 	k_sleep(500);

			// 	uart_tx(uartGsm, status, sizeof(status), 50);
			// 	uart_rx_enable(uartGsm, response1, sizeof(response1), 50);
			// 	uart_tx(uartSerial, response1, sizeof(response1), 50);

			// 	break;
			// }

			// uart_tx(uartGsm, gprsStatus, sizeof(gprsStatus), 100);
			// uart_tx_abort(uartGsm);
			// k_sleep(100);
			// uart_rx_enable(uartGsm, response, sizeof(response), 20);
			// uart_tx(uartSerial, response, sizeof(response), 20);
			// uart_tx(uartGsm, setConnectionType, sizeof(setConnectionType), 50);
			// k_sleep(100);
			// uart_rx_enable(uartGsm, response, sizeof(response), 50);
			// uart_tx(uartSerial, response, sizeof(response), 50);
			// k_sleep(100);

			// uart_tx(uartGsm, apnConnect, sizeof(apnConnect), 50);
			// uart_rx_enable(uartGsm, response, sizeof(response), 50);
			// uart_tx(uartSerial, response, sizeof(response), 50);
			// k_sleep(100);

			// uart_tx(uartGsm, username, sizeof(username), 50);
			// uart_rx_enable(uartGsm, response, sizeof(response), 50);
			// uart_tx(uartSerial, response, sizeof(response), 50);
			// k_sleep(100);

			// uart_tx(uartGsm, passwordSet, sizeof(passwordSet), 50);
			// uart_rx_enable(uartGsm, response, sizeof(response), 50);
			// uart_tx(uartSerial, response, sizeof(response), 50);
			// k_sleep(100);

			// uart_tx(uartGsm, definePdpContext, sizeof(definePdpContext), 50);
			// uart_rx_enable(uartGsm, response, sizeof(response), 50);
			// uart_tx(uartSerial, response, sizeof(response), 50);
			// k_sleep(600);

			// uart_tx(uartGsm, connectPdp, sizeof(connectPdp), 50);
			// uart_rx_enable(uartGsm, response, sizeof(response), 50);
			// uart_tx(uartSerial, response, sizeof(response), 50);
			// k_sleep(85000);

			// uart_tx(uartGsm, openDefinedGprsContext, sizeof(openDefinedGprsContext), 50);
			// uart_rx_enable(uartGsm, response, sizeof(response), 50);
			// uart_tx(uartSerial, response, sizeof(response), 50);
			// k_sleep(3500);

			// uart_tx(uartGsm, queryGprsContext, sizeof(queryGprsContext), 50);
			// uart_rx_enable(uartGsm, response, sizeof(response), 50);
			// uart_tx(uartSerial, response, sizeof(response), 50);
			// k_sleep(6000);

			// uart_tx(uartGsm, connectGprs, sizeof(setConnectionType), 50);
			// k_sleep(100);
			// uart_rx_enable(uartGsm, response8, sizeof(response8), 50);
			// uart_tx(uartSerial, response8, sizeof(response8), 50);
			// k_sleep(6000);

			// uart_tx(uartGsm, setMultiIP, sizeof(setMultiIP), 50);
			// k_sleep(100);
			// uart_rx_enable(uartGsm, response9, sizeof(response9), 50);
			// uart_tx(uartSerial, response9, sizeof(response9), 50);
			// k_sleep(1000);

			// uart_tx(uartGsm, quickSend, sizeof(quickSend), 50);
			// k_sleep(100);
			// uart_rx_enable(uartGsm, response10, sizeof(response10), 50);
			// uart_tx(uartSerial, response10, sizeof(response10), 50);
			// k_sleep(1000);

			// uart_tx(uartGsm, setGetDataManual, sizeof(setGetDataManual), 50);
			// k_sleep(100);
			// uart_rx_enable(uartGsm, response11, sizeof(response11), 50);
			// uart_tx(uartSerial, response11, sizeof(response11), 50);
			// k_sleep(1000);

			// uart_tx(uartGsm, startTask, sizeof(startTask), 50);
			// k_sleep(100);
			// uart_rx_enable(uartGsm, response12, sizeof(response12), 50);
			// uart_tx(uartSerial, response12, sizeof(response12), 50);
			// k_sleep(600);

			// uart_tx(uartGsm, startConnection, sizeof(startConnection), 50);
			// k_sleep(100);
			// uart_rx_enable(uartGsm, response13, sizeof(response13), 50);
			// uart_tx(uartSerial, response13, sizeof(response13), 50);
			// k_sleep(600);

			// uart_rx_disable(uartGsm);
		}
		// const char *st = "48656C6C6F3B";
		// int length = strlen(st);
		// int i;
		// char buf = 0;
		// for (i = 0; i < length; i++)
		// {
		// 	if (i % 2 != 0)
		// 	{
		// 		int asc = hex_to_ascii(buf, st[i]);
		// 		char ascii = asc;
		// 		uart_tx(uartSerial, ascii, sizeof(ascii), 50);
		// 		// printf("%c", hex_to_ascii(buf, st[i]));
		// 	}
		// 	else
		// 	{
		// 		buf = st[i];
		// 	}
		// }

		// u8_t subscribe[200] = {0};
		// uart_rx_enable(uartGsm, subscribe, sizeof(subscribe), 50);
		// // if(subscribe[0]=='+' && subscribe[1]=='C')
		// // {

		// // }
		// uart_tx(uartSerial, subscribe, sizeof(subscribe), 50);
#endif
	}

	// char *st = "48656C6C6F3B";
	// *st = *asc;
	// int length = strlen(st);
	// int i;
	// char buf = 0;
	// for (i = 0; i < length; i++)
	// {
	// 	if (i % 2 != 0)
	// 	{
	// 		int as = hex_to_ascii(buf, st[i]);
	// 		char ascii = as;
	// 		uart_tx(uartSerial, ascii, sizeof(ascii), 50);
	// 		// printf("%c", hex_to_ascii(buf, st[i]));
	// 	}
	// 	else
	// 	{
	// 		buf = st[i];
	// 	}
	// }
}
