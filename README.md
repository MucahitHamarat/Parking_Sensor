# STM32F103C8, LCD 16x2 , HC-SR04 bağlantılar hakkında

LCD'nin bağlantıları birebir Proteustan alınan ekran görüntüsündeki gibidir(proje dosyasındaki png dosyası). 5k ohm ile LCD 
kontrastı değiştirilebiliyor. 

Ultrasonic Sensör'ün VCC ucu 5v ile beslenen Breadboard'un + ucuna, 
GND ucu aynı breadboardun - ucuna direkt bağlıdır. 

U. Sensör'ün Trig ucu , STM32'nin B4 bacağında direkt bağlıdır, 
Echo pini ise birbirine seri bağlı 2 adet 470 ohm'luk dirençler ile GND ye gidiyor, bu 2 direncin ortasından
STM32'nin B3 pinine bir bağlantı yapılıyor. 

