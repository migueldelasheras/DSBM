Modo de ejecución(Windows):

Para la ejecución del programa primero deberemos iniciar el broker. Para ello utilizaremos el comando:
"C:\Program Files\mosquitto\mosquitto.exe" -c broker.conf

A continuación iniciaremos el suscriptor que será quien reciba todos los mensajes enviados por la placa ESP32. Para ello el comando utilizado será: 
"C:\Program Files\mosquitto\mosquitto_sub.exe" -t A12/# -h 192.168.43.174 -p 2883