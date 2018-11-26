Overview
========
The Multicore RPMsg-Lite pingpong project is a simple demonstration program that uses the
MCUXpresso SDK software and the RPMsg-Lite library and shows how to implement the inter-core
communicaton between cores of the multicore system. The primary core releases the secondary core
from the reset and then the inter-core communication is established. Once the RPMsg is initialized
and endpoints are created the message exchange starts, incrementing a virtual counter that is part
of the message payload. The message pingpong finishes when the counter reaches the value of 100.
Then the RPMsg-Lite is deinitialized and the procedure of the data exchange is repeated again.

Shared memory usage
This multicore example uses the shared memory for data exchange. The shared memory region is
defined and the size can be adjustable in the linker file. The shared memory region start address
and the size have to be defined in linker file for each core equally. The shared memory start
address is then exported from the linker to the application.
Hardware requirements
=====================
- Mini/micro USB cable
- LPCXpresso55s69 board
- Personal Computer

Board settings
==============
The Multicore RPMsg-Lite pingpong project does not call for any special hardware configurations.
Although not required, the recommendation is to leave the development board jumper settings and 
configurations in default state when running this demo.


Prepare the Demo
================
1.  Connect a micro USB cable between the PC host and the CMSIS DAP USB port (J7) on the board
2.  Open a serial terminal with the following settings (See Appendix A in Getting started guide for description how to determine serial port number):
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.

For detailed instructions, see the appropriate board User's Guide.

Running the demo
================
The log below shows the output of the RPMsg-Lite pingpong demo in the terminal window:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Copy CORE1 image to address: 0x20033000, size: 9952

RPMsg demo starts
Primary core received a msg
Message: Size=4, DATA = 1
Primary core received a msg
Message: Size=4, DATA = 3
Primary core received a msg
Message: Size=4, DATA = 5
.
.
.
Primary core received a msg
Message: Size=4, DATA = 101

RPMsg demo ends
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Toolchain supported
===================
- GCC ARM Embedded 
- Keil MDK 
- IAR embedded Workbench 
- MCUXpresso


��������չIO����ʹ�������
================
P17
        1       2
        3       4
        5       6 FC1_TXD 
        7       8
        9       10 FC7_RXD
        11      12 FC7_TXD
        13      14
        15      16
        17      18
        19      20
        
P18
        1       2
FC1_RXD 3       4
        5       6
        7       8
        9       10
        11      12
FC2_TXD 13      14
FC2_RXD 15      16
        17      18
        19      20

������Э�飺
================
��Դ5V��ͨѶ3.3V
������Э�飬����ASCII�룬����"p0.pic=1"(�ر�)����"p0.pic=2"(��)�����ƽ�����ʾ����״̬��
������������ʱ�����ڻᷢ��0x01(�ر�)����0x02(��)
���ڲ�����9600

��������Э��
================
����Э�飬������ʹ��HC-08����ģ�飬��Դ3.3V������ͨѶ
���ڲ�����Ϊ9600
���յ�0x30���������յ�0x31����
ͬ������ʱͬ������0x31������ʱͬ��0x30

WIFI����Э��
================
ģ��ʹ��ESP-01S����Դ3.3V������ͨѶ��������115200.
�����Ĳ�����Ҫ����
�������Ӻͽ����Ȩ��
�жϽ�����Ҫ����
��ʱ����״̬
״̬�ı估ʱͬ��


��ѭ���߼�
================
�����Դ����Ƿ��յ����ݣ�����н��д���
���WiFi�����Ƿ��յ����ݣ�����н��д���
������������Ƿ��յ����ݣ�����н��д���
���LCD�����Ƿ��յ����ݣ�����н��д���
��ʱ�������ݵ����Դ���
��ʱ�������ݵ���������
��ʱ�������ݵ�WiFi����
��ʱ�������ݵ�LCD����
�����״̬������ֵ�����ݸ�ֵ���д���
�����Ҫͬ������ͬ����Ϣ��LCD��WIFI��Debug���ڵȵ�
��ʱ10ms

�ļ��޸��б�
================
1��board/pin_mux.h��pin_mux.h����Ҫ����������IO�Ĺ��ܣ�����ʹ�õ���LED�ƺ��ĸ�����
2��doc/readme.txt����Ҫ����˵�������̵�һЩ��Ϣ��
3��Source/Common.h����Ҫ�Ƕ����õ���һЩ��������
4��Source/MqttKit.h��MqttKit.c�������������ӵ�OneNET�Ʒ�������Э������ļ�����ֲ��OneNET�ٷ�����
5��Source/smart_lock.c�����̵���Ҫ�ļ���ʵ�����������ӣ����Դ������ӣ�wifi���ӵ��Ʒ�������

ʵ�ֵ���Ҫ����
================
1������ͨ�����Դ��ڽ��п�����״̬��
2������ͨ��LCD��Ļ������״̬��
3������ͨ������������״̬��
4������ͨ��Զ��web�����ֻ�app������״̬��
5�����п���;��������״̬ͬ����������webҳ���޸�״̬����LCD�������ϻ�ͬ��״̬��

�������Ĺ���
================
1������smart_lock.c�ļ�������ע�ͺʹ����ʽ����
2��ʵ��˫�˰汾�ĳ�����Ҫ������core1ʵ��led�ƵĿ��Ƽ��ɡ�