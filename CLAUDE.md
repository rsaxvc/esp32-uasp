We're building a UASP (USB-Attached-SCSI-Protocol) driver for ESP32-S3 using ESP-IDF6

Project Layout:
* build/ - build stuff
* main/ - our main code for this project
* managed_components/ - components downloaded by ESP-IDf

Hardware:
* ESP32-S3-WROOM-1 with 8MB of RAM and 16MB of Flash
* Serial port for flashing and console monitoring
* USB port that will run UASP
* We'll use the remaining flash memory for a virtual disk, using the ESP-IDF's wear-level library.

Other tools:
* I can give you sudo access to specific commands if you ask

Please feel free to ask questions.
