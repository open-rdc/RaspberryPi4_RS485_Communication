# RaspberryPi4_RS485_Communication
RaspberryPi4のUARTを5つ使いRS485の並列通信を行う

回路図

![5serialtest_回路図ver2](https://user-images.githubusercontent.com/5755200/147544493-d849f4c5-3b15-4f72-a5d2-fcf25d883e05.png)

UARTポートに対応する送受信切り替えPinの詳細


UART | tty | GPIO | wiringPi | pin
-- | -- | -- | -- | --
0 | AMA0 | 18 | 1 | 12
2 | AMA1 | 27 | 2 | 13
3 | AMA2 | 22 | 3 | 15
4 | AMA3 | 19 | 24 | 35
5 | AMA4 | 21 | 29 | 40
