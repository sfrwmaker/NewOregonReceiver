# NewOregonReceiver
Another Oregon v2 protocol receiver library. This variant mostly uses interrupt handler to free CPU resources in main Arduino loop.
While in WirelessOregonV2 library the interrupt handler checks the pulse length only, this library interrupt handler manages the sensor signal completely.
To chack the signa was received, use isMessageReceived() method. The received message kept in the receive buffer till receiveData() called.
The sketch can do other job and check the message received with a longer period.



