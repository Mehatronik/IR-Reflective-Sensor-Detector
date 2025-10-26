Code for nRF52840 (SDK 17.1) and Infra-red detector sensor CNY70, which is a very simple IR diode + IR photo-transistor pair in one package.
Therefore, this is applicable to any similar sensor.

Sensor is powered by 3V3.
IR LED is toggled (at 50Hz - not specifically choosen) wiht the help of a driver transistor, as GPIO can't handle more than few mA, while IR
LED is set to pull 20mA.

Output of the photo-transistor (10k on Emitter) is positive, and fed into the ADC of nRF.

Logic for detection is simple: 
- Turn IR diode OFF, start ADC and collect samples
- Turn IR diode ON, start ADC and collect samples
- Compare samples when ON and OFF -> if there was no measurable difference, it means that there was no object. If there was a difference, 
  count at least some X consecutive number of samples and consider object detected.

  Since we're measuring the differene in OFF vs ON state, this means that IR sensor could be bombarded with background IR, and have a 
  significant bias on the output, but controller will still correctly report if object is detected or not.

- Note: IR sensor is best used with IR filter glass (blocking visible spectrum), as strong visible light (e.g. flash light) causes significant
  impact. But the glass will cause reflection, so some "dead zone" must be added when detecting the actual object.
