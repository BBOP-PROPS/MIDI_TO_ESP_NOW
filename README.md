# MIDI_TO_ESP_NOW
Intercept MIDI Keyboard Key Press and cause an action on a remote ESP Device.  

[ESP-NOW](https://www.espressif.com/en/products/software/esp-now/overview)

[ESP-NOW Example](https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/)

[MIDI with Arduino](https://www.instructables.com/Send-and-Receive-MIDI-with-Arduino/)

![MIDI](https://www.sweetwater.com/insync/media/2020/07/Akai-Professional-MPK-Mini-mkII-25-key-Keyboard-Controller-1536x950.jpg)
![ESP-NOW](https://i0.wp.com/randomnerdtutorials.com/wp-content/uploads/2020/01/ESP_NOW_one_master_multiple_slaves.png?w=652&quality=100&strip=all&ssl=1)

When a student plays a key on the MIDI Keyboard, it should send a remote action throuh the ESP-NOW protocol.  Pressing the keyboard "C" key might turn on a light momentarily on a remote prop.  "D" might turn on a stepper motor on a different prop.

The "Master" will interface with the MIDI keyboard, and the majority of the programming will be on the Slaves.
