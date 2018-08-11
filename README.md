# GPS
Read Serial GPS NEMA returs a Lat Lon pair in a string.

CLI params select between (Lon Lat), Alt, (Speed heading), status data and a free running input directly sent to the output.

To Build
gcc -Wall main.c -o gps

To Run
sudo ./gps
(I can't be arsed to sort out the acces for the serial port)
